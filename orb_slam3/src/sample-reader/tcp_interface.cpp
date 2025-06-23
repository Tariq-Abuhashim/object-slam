// Copyright (c) 2023 Mission Systems Pty Ltd
// Tariq Abuhashim, May 2023

/*
TCP-Interface for Object-SLAM
to compile: g++ -o tcp_interface tcp_interface.cpp -lboost_system -lpthread
*/

#include <iostream>
#include <queue>
#include <string>
#include <thread>
#include <mutex>
#include <boost/asio.hpp>

#include <snark/imaging/cv_mat/serialization.h>
#include <snark/sensors/lidars/ouster/types.h>
#include <snark/sensors/lidars/ouster/config.h>
#include <snark/sensors/lidars/ouster/traits.h>

const std::size_t IMAGE_SIZE = 15197972;
const double EPSILON = 0.01;
std::mutex mtxCloudQueue, mtxImageQueue;

std::uint64_t to_microseconds(const boost::posix_time::ptime& t) {
    static boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
    return (t - epoch).total_microseconds();
}

struct LidarPoint {
	// t,2uw,2ui,uw,3d,3uw,3d
	// ouster-to-csv lidar --output-format | csv-format collapse
	// ouster-to-csv lidar --output-fields
    std::uint64_t timestamp; // 8-bytes (64-bits)
	std::uint16_t measurement_id; // 2-bytes
	std::uint16_t frame_id; // 2-bytes
	std::uint32_t encoder_count; // 4-bytes
	std::uint32_t block; // 4-bytes
	std::uint16_t channel; // 2-bytes
	double range; // 8-bytes (64-bits)
	double bearing; // 8-bytes (64-bits)
	double elevation; // 8-bytes (64-bits)
	std::uint16_t signal; // 2-bytes
	std::uint16_t reflectivity; // 2-bytes
	std::uint16_t ambient; // 2-bytes
	double point[3]; // 24-bytes (x,y,z)
};

//typedef std::vector<LidarPoint> LidarScan;

struct Header {
    uint32_t seq;
    std::uint64_t stamp; // ROS uses a specific time format, but we can use a double for simplicity.
    std::string frame_id;
};

struct Fields {
    std::uint8_t INT8    = 1;
	std::uint8_t UINT8   = 2;
	std::uint8_t INT16   = 3;
	std::uint8_t UINT16  = 4;
	std::uint8_t INT32   = 5;
	std::uint8_t UINT32  = 6;
	std::uint8_t FLOAT32 = 7;
	std::uint8_t FLOAT64 = 8;

	std::string name;      // Name of field
	std::uint32_t offset;    // Offset from start of point struct
	std::uint8_t datatype;  // Datatype enumeration, see above
	std::uint32_t count;    // How many elements in the field

	void clear() {
		name.clear();// std::string has a clear function that resets it to an empty string
        offset = 0;
		datatype = 0;
		count = 0;
    }
};

struct Cloud {
	Header header;
	uint32_t height;
	uint32_t width;
	Fields fields;
	bool is_bigendian;
	uint32_t point_step;
	uint32_t row_step;
	std::uint16_t* data;
	bool is_dense;

	void clear() {
        header.stamp = 0;
		header.frame_id.clear();// std::string has a clear function that resets it to an empty string
        height = 0;
		width = 0;
		fields.clear();
		is_bigendian = false;
		point_step = 0;
		row_step = 0;
		data = nullptr;
		is_dense = true;
    }

	bool isempty() {
		return width==0;
	}

	std::size_t size() {
		return width;
	}
};

struct Image {
	Header header;
 	uint32_t height;
    uint32_t width;
    std::string encoding;
    bool is_bigendian;
    uint32_t step;
    std::vector<uint8_t> data; // Use a vector to store the image data because the size can vary.
};

//template <typename T>
//using SensorData = std::pair<double, T>;
template <typename T>
class SensorData {
public:
    SensorData(double timestamp, const T& data)
        : timestamp_(timestamp), data_(data) {}

    double getTimestamp() const { return timestamp_; }
    const T& getData() const { return data_; }

private:
    double timestamp_;
    T data_;
};

template <typename T>
using dataQueue = std::queue<SensorData<T>>;

/*template <typename T>
void addData(std::queue<SensorData<T>>& queue, double timestamp, const T& reading) {
    queue.push(std::make_pair(timestamp, reading));
}*/

template <typename T1, typename T2>
void processAndSyncData(std::queue<SensorData<T1>>& queue1, std::queue<SensorData<T2>>& queue2,
    const std::function<void(const SensorData<T1>&, const SensorData<T2>&)>& func) {
	//std::cout << queue1.size() << " " << queue2.size() << std::endl;
    while (!queue1.empty() && !queue2.empty()) {
		std::lock_guard<std::mutex> guard_image(mtxImageQueue);
		std::lock_guard<std::mutex> guard_cloud(mtxCloudQueue);
        SensorData<T1> data1 = queue1.front();
        SensorData<T2> data2 = queue2.front();
        if (std::abs(data1.getTimestamp()/1e+9 - data2.getTimestamp()/1e+9) < EPSILON) {
            // Call the passed function with the synchronized data
            func(data1, data2);
            queue1.pop();
            queue2.pop();
        }
        else if (data1.getTimestamp()/1e+9 < data2.getTimestamp()/1e+9) {
            queue1.pop();
        }
        else {
            queue2.pop();
        }
    }
}

template <typename T>
class SensorReader {
public:
    SensorReader(std::queue<SensorData<T>>& queue, const std::string& ip, short port, const std::string& format)
        : queue_(queue), ip_(ip), port_(port), socket_(nullptr), format_(format) {}

    void readFromSocket() {
		try {
		    boost::asio::io_service io_service;
            socket_ = new boost::asio::ip::tcp::socket(io_service);
            boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(ip_), port_);
            socket_->connect(endpoint);
		    while (true) { handleData(queue_); }
		}
		catch (const boost::system::system_error& ex) {
			std::cerr << ex.what() << '\n';
            std::cerr << ex.code() << '\n';
		}
    }

private:

	void handleData(std::queue<SensorData<T>>& queue_) {
		processImpl(queue_);
	}

	/* Image specific function 
		In C++, std::enable_if and std::is_same are commonly used in template metaprogramming to control the 
			instantiation of templates based on type traits or properties of types.
		Few reasons why you might want to create type-specific templates:
		1- Type-specific behavior: If you have a template function or class that should behave differently 
			depending on the type of its template arguments, you can use std::enable_if and std::is_same 
			to create different specializations for different types. This can often be more convenient 
			and maintainable than using runtime type checks or dynamic casting.
		2- Type safety: By using type-specific templates, you can ensure that your code will only compile 
			with types that make sense. For example, if you have a template function that works with numerical 
			types, you can use std::enable_if and type traits like std::is_arithmetic to ensure that the 
			function can't be instantiated with non-numerical types.
		3- Optimization: Sometimes, certain types can be processed more efficiently with different algorithms. 
			By using type-specific templates, you can automatically select the most efficient algorithm based 
			on the type.
	*/
	template <typename U = T>
	typename std::enable_if<std::is_same<U, Image>::value>::type processImpl(std::queue<SensorData<T>>& queue) {
        // Processing specific to Image data type here...
		boost::asio::streambuf buffer;
		boost::asio::read(*socket_, buffer, boost::asio::transfer_exactly(IMAGE_SIZE));  // FIXME compute size
		Image reading = handleImageData(buffer);
		double timestamp = reading.header.stamp;
		addData(queue, timestamp, reading);
    }

	/* Cloud specific function */
	template <typename U = T>
	typename std::enable_if<std::is_same<U, Cloud>::value>::type processImpl(std::queue<SensorData<T>>& queue) {
    	// Processing specific to Cloud data type here...
		LidarPoint lidar_point;
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.timestamp, sizeof(lidar_point.timestamp)));
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.measurement_id, sizeof(lidar_point.measurement_id)));
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.frame_id, sizeof(lidar_point.frame_id)));
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.encoder_count, sizeof(lidar_point.encoder_count)));
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.block, sizeof(lidar_point.block)));
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.channel, sizeof(lidar_point.channel)));
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.range, sizeof(lidar_point.range)));
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.bearing, sizeof(lidar_point.bearing)));
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.elevation, sizeof(lidar_point.elevation)));
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.signal, sizeof(lidar_point.signal)));
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.reflectivity, sizeof(lidar_point.reflectivity)));
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.ambient, sizeof(lidar_point.ambient)));
		boost::asio::read(*socket_, boost::asio::buffer(&lidar_point.point[0], sizeof(lidar_point.point)));

		if (lidar_point.reflectivity==0) return; // FIXME filter points before adding them, which field to use?

		if (currentFrameId!=lidar_point.frame_id) { // check if this is a new scan // FIXME seems like block is more stable to use then frame_id
			if (currentFrameId!=0) {
				//std::cout << "Number of points in scan " << currentFrameId << " = " << cloud.size() << std::endl;
				if (!cloud.isempty()) addData(queue, cloud.header.stamp, cloud);
			}
			//lidar_scan.clear();
			//lidar_scan.resize(0);

			currentFrameId = lidar_point.frame_id;
			
			cloud.clear();
			Header header;
			header.stamp = lidar_point.timestamp;
			header.frame_id = "ouster_frame";
			cloud.header = header;
			cloud.height = 1;
		}
		cloud.width+=1;
		//cloud.fields;  //FIXME
		//cloud.data;  //FIXME
		//lidar_scan.push_back(lidar_point);
	}

	/* General function */
	template <typename O>
	void addData(std::queue<SensorData<O>>& queue, double timestamp, const O& data) {
		if (std::is_same<T, Cloud>::value)
			std::lock_guard<std::mutex> guard(mtxCloudQueue);
		else if (std::is_same<T, Image>::value)
			std::lock_guard<std::mutex> guard(mtxImageQueue);
		else
			std::cerr << "addData: Unkown types" << std::endl;
		
		while (!queue.empty() && queue.back().getTimestamp() > timestamp) {
		    queue.pop();
		}
		queue.push(SensorData<O>(timestamp, data));
	}
	
	/* Image specific function */
	Image handleImageData(boost::asio::streambuf& buffer) {
		std::istream is(&buffer);
		snark::cv_mat::serialization i;
    	std::pair< boost::posix_time::ptime, cv::Mat > time_image = i.read< boost::posix_time::ptime >(is);
		// Create a header
		Header header;
		header.seq = 1;
		header.stamp = to_microseconds(time_image.first);  // This should be a meaningful timestamp in your application.
		header.frame_id = "camera_frame";
		// Initialize the Image
		Image img;
		img.header = header;
		img.height = time_image.second.rows;
		img.width = time_image.second.cols;
		img.encoding = "bgr8";
		img.is_bigendian = false;
		img.step = time_image.second.step[0];  // Assuming 3 bytes per pixel for BGR8 encoding.
		img.data.assign(time_image.second.datastart, time_image.second.dataend);
    	//std::cout << "Received Image Data: Timestamp: " << header.stamp
        //      	<< ", Dimensions: " << img.width << "x" << img.height
        //      	<< ", Type: " << time_image.second.type()
        //      	<< ", Pixel Count: " << img.width*img.height*time_image.second.channels() << std::endl;
		return img;
	}

private:
    std::queue<SensorData<T>>& queue_;
    std::string ip_;
    short port_;
    boost::asio::ip::tcp::socket* socket_;
	const std::string format_;
	//LidarScan lidar_scan;
	Cloud cloud;
	std::uint16_t currentFrameId = 0;
};

// Example function to process synchronized data
template <typename T1, typename T2>
void processSyncedData(const SensorData<T1>& data1, const SensorData<T2>& data2) {
    // Replace this with your actual processing code
    std::cout << "Processing data: Lidar time(" << data1.getTimestamp()/1e9 << "), Image time(" << data2.getTimestamp()/1e9 << ")" << std::endl;
}

int main() {

	std::cout.precision(12);

	// Initialise readers
	dataQueue<Cloud> lidarData;
	dataQueue<Image> imageData;
    SensorReader<Cloud> reader1(lidarData, "127.0.0.1", 4000, "t,2uw,2ui,uw,3d,3uw,3d"); // FIXME formats are not used
    SensorReader<Image> reader2(imageData, "127.0.0.1", 4001, "t,3ui,s[15197952]");

    // Start threads to read from the sensors
    std::thread t1(&SensorReader<Cloud>::readFromSocket, &reader1);
    std::thread t2(&SensorReader<Image>::readFromSocket, &reader2);

    // Process and synchronize the data in the main thread
    while (true) {
        processAndSyncData(lidarData, imageData, processSyncedData<Cloud, Image>);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Sleep to avoid busy waiting
    }

    // Join the threads when done (this code is never reached in this example)
    t1.join();
    t2.join();

    return 0;
}