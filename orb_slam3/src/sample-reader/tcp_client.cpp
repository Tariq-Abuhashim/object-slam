
// Copyright (c) 2023 Mission Systems Pty Ltd
// Tariq Abuhashim, May 2023

/*
TCP-Interface for Object-SLAM
to compile: g++ -o tcp_client tcp_client.cpp -lboost_system -lpthread
*/

#include <iostream>
#include <vector>
#include <thread>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/name_value/serialize.h> // read_json
//#include <comma/csv/stream.h>
//#include <comma/io/select.h>
//#include <comma/io/stream.h>
//#include <comma/visiting/traits.h>
//#include <comma/io/file_descriptor.h>

#include <snark/imaging/cv_mat/serialization.h>
#include <snark/sensors/lidars/ouster/types.h>
#include <snark/sensors/lidars/ouster/config.h>
#include <snark/sensors/lidars/ouster/traits.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense> // Vector3d for lidar point x,y,z

using boost::asio::ip::tcp;

static const std::string default_config( "config.json:/home/mrt/data/mission_systems/format_testing/configs/" );

struct intrinsics_t
{
    snark::ouster::lidar::transform_t imu_transform;
    snark::ouster::lidar::transform_t lidar_transform;

    intrinsics_t() {}

    intrinsics_t( snark::ouster::lidar::config_t& config )
        : imu_transform( config.imu_intrinsics.imu_to_sensor_transform )
        , lidar_transform( config.lidar_intrinsics.lidar_to_sensor_transform )
    {}
};
static intrinsics_t intrinsics;
static snark::ouster::lidar::beam_angle_lut_t beam_angle_lut;

// echo t,3ui | csv-format size    return 20 for images
// FIXME these need to be calculated from input format
const std::size_t IMAGE_HEADER_SIZE = 20; // Assuming the header contains timestamp (8 bytes) and image dimensions (8 bytes)
const std::size_t IMAGE_SIZE = 15197952; 
const std::size_t LIDAR_HEADER_SIZE = 22;
const std::size_t LIDAR_SIZE = 54;

struct ImageData {
	// t,3ui,s[15197952] - total image size is 15197972-bytes
	// echo t,3ui | csv-format size    return 20-bytes for image header
    std::uint64_t timestamp; // 8-bytes (64-bits)
    std::uint32_t width; // 4-bytes (32-bits)
    std::uint32_t height; // 4-bytes
	std::uint32_t type; // 4-bytes
    std::vector<std::uint8_t> pixels; // 1-byte (8-bits) x 15197952 pixels  FIXME type std::uint8_t is hard-coded here
};

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

typedef std::vector<LidarPoint> LidarScan;

/* 	calculates the difference between the boost::posix_time::ptime argument and the epoch.
   	calling total_microseconds() on the difference gives the total number of microseconds (in std::uint64_t).
 	Thi code will only work correctly for times after 1970-01-01 00:00:00 UTC. 
	If you need to handle times before the Unix epoch, you'll need to use a different epoch and adjust the code accordingly.
*/
std::uint64_t to_microseconds(const boost::posix_time::ptime& t) {
    static boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
    return (t - epoch).total_microseconds();
}

/* 	cv::Mat objects do not have a direct method to get the data type of the stored elements.
	However, you can use the type() method to get an integer representing the type, 
	and then use the CV_MAT_DEPTH macro to extract the depth (which represents the data type) from this integer.
	example: type2str(img.type())
*/
std::string type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

void handleImageData(const ImageData& imageData) {
    // Process the received image data as per your requirements
    std::cout << "Received Image Data: Timestamp: " << imageData.timestamp
              << ", Dimensions: " << imageData.width << "x" << imageData.height
              << ", Type: " << imageData.type
              << ", Pixel Count: " << imageData.pixels.size() << std::endl;
}

void handleImageData(const std::pair< boost::posix_time::ptime, cv::Mat >& p) {
    // Process the received image data as per your requirements
    std::cout << "Received Image Data: Timestamp: " << to_microseconds(p.first)
              << ", Dimensions: " << p.second.cols << "x" << p.second.rows
              << ", Type: " << p.second.type()
              << ", Pixel Count: " << p.second.cols*p.second.rows*p.second.channels() << std::endl;
}

void readHeader(boost::asio::ip::tcp::socket& socket, boost::asio::streambuf& receiveBuffer, const std::size_t& size) {
	try {
    	boost::asio::read(socket, receiveBuffer, boost::asio::transfer_exactly(size));
	} catch (const std::exception& ex) {
		std::cerr << "readHeader Error: " << ex.what() << std::endl;
	}
}

void readPixelData(boost::asio::ip::tcp::socket& socket, boost::asio::streambuf& receiveBuffer, const std::size_t& size) {
	try {
    	boost::asio::read(socket, receiveBuffer, boost::asio::transfer_exactly(size));
	} catch (const std::exception& ex) {
		std::cerr << "readPixelData Error: " << ex.what() << std::endl;
	}
}


static snark::ouster::lidar::config_t load_config( )
{
    std::vector< std::string > config_components = comma::split( default_config , ':' );
    std::string config_filename = config_components[0];
    std::string config_path = ( config_components.size() > 1 ? config_components[1] : "" );
    std::cout << "reading " << config_filename << " with path " << config_path << std::endl;
    return comma::read_json< snark::ouster::lidar::config_t >( config_path+config_filename, config_path );
}

void usage( bool )
{
    std::cerr << "\nSample application for reading binary data streams";
    std::cerr << "\n";
    std::cerr << "\nMakes some major assumptions about the input data";
    std::cerr << "\n";
    std::cerr << "\nUsage: tcp_client <list-of-ports>";
    std::cerr << "\n";
    std::cerr << "\nOptions:";
    std::cerr << "\n    --help,-h:     display this help message and exit";
    std::cerr << "\n    --verbose,-v:  more output";
    std::cerr << "\n";
    std::cerr << "\nExample:";
    std::cerr << "\n    ./tcp_client '4000;binary=t,2uw,2ui,uw,3d,3u3d' '4001;binary=t,3ui,s[15197952]'";
    std::cerr << "\n" << std::endl;
}

size_t get_size(std::string format)
{
	std::vector< std::string > v = comma::split( format, ",%" );
    std::size_t offset = 0, size_ = 0;
	
    for( unsigned int i = 0; i < v.size(); ++i )
    {
        std::string s;
        for( ; s.length() < v[i].length() && v[i][ s.length() ] >= '0' && v[i][ s.length() ] <= '9'; s += v[i][ s.length() ] );
        //if( s.length() >= v[i].length() ) { COMMA_THROW( comma::exception, "expected format, got '" << v[i] << "' in " << format ); }
        std::size_t arraySize = s.empty() ? 1 : boost::lexical_cast< std::size_t >( s );
        std::string type = v[i].substr( s.length() );
        unsigned int size;
        if( type == "b" ) { size = 1; }
        else if( type == "ub" ) { size = 1; }
        else if( type == "w" ) { size = 2; }
        else if( type == "uw" ) { size = 2; }
        else if( type == "i" ) { size = 4; }
        else if( type == "ui" ) { size = 4; }
        else if( type == "l" ) { size = 8; }
        else if( type == "ul" ) { size = 8; }
        else if( type == "c" ) { size = 1; }
        else if( type == "t" ) { size = sizeof( std::uint64_t ); }
        else if( type == "lt" ) { size = sizeof( std::uint32_t ) + sizeof( std::uint64_t ); }
        else if( type == "f" ) { size = sizeof( float ); }
        else if( type == "d" ) { size = sizeof( double ); }
        //else if( type[0] == 's' && type.length() == 1 ) { COMMA_THROW( comma::exception, "got variable size string in [" << format << "]: not implemented, use fixed size string instead, e.g. \"s[8]\"" ); }
        else if( type[0] == 's' && type.length() > 3 && type[1] == '[' && *type.rbegin() == ']' )
        { size = boost::lexical_cast< std::size_t >( type.substr( 2, type.length() - 3 ) ); }
        //else { COMMA_THROW( comma::exception, "expected format, got '" << type << "' in " << format ); }
        //elements_.push_back( element( offset, arraySize, size, t ) );
        //count_ += arraySize;
        size *= arraySize;
        offset += size;
        size_ += size;
    }
	return size_;
}

// this class handles the sockets
class connection
{
public:
	connection(boost::asio::io_service& service, const std::string& server, const std::pair<std::string, std::string>& input )
		: resolver_(service), socket_(service), port_(input.first), format_(input.second), size_(get_size(input.second))
	{

		//std::cout << "Message setting message size to " << get_size( format_ ) << std::endl;

		// Resolve the endpoint to connect to
		//tcp::resolver resolver_(io_service); // step moved to constructor
		tcp::resolver::query query(server, port_);
		tcp::resolver::iterator endpoint_iterator = resolver_.resolve( query );
		// Create a socket and connect to the endpoint
		//tcp::socket socket(io_service);  // step moved to constructor
		try {
			boost::asio::connect(socket_, endpoint_iterator);
		} catch (const std::exception& ex) {
			std::cerr << "Connection Error: " << ex.what() << std::endl;
		}
/*
		tcp::iostream* s = new tcp::iostream( endpoint_iterator->endpoint() );
		//if( !*s ) { delete s; COMMA_THROW( comma::exception, "failed to connect to " << name << ( blocking_ ? " (todo: implement blocking mode)" : "" ) ); }
		close_ = boost::bind( &tcp::iostream::close, s );
        // todo: make unidirectional
#if (BOOST_VERSION >= 106600)
        fd_ = s->rdbuf()->native_handle();
#else
        fd_ = s->rdbuf()->native();
#endif
*/

	}

	std::pair< boost::posix_time::ptime, cv::Mat > processReceivedData(boost::asio::streambuf& receiveBuffer) 
	{
		std::istream inputStream(&receiveBuffer);
		snark::cv_mat::serialization input;
        // Display image
        //cv::imshow("Image", p.second);
        //cv::waitKey(5);
		//std::cout << type2str(p.second.type()) << std::endl;
        return input.read< boost::posix_time::ptime >(inputStream);
	}
/*
	ImageData processReceivedData(boost::asio::streambuf& receiveBuffer) 
	{
		std::istream inputStream(&receiveBuffer);

		// Read the timestamp and image dimensions from the header
		std::uint64_t timestamp;
		std::uint32_t width, height, type;
		boost::asio::read(socket_, boost::asio::buffer(&timestamp, sizeof(timestamp)));
        boost::asio::read(socket_, boost::asio::buffer(&width, sizeof(width)));
        boost::asio::read(socket_, boost::asio::buffer(&height, sizeof(height)));
		boost::asio::read(socket_, boost::asio::buffer(&type, sizeof(type)));

		// Read image data
		cv::Mat img = cv::Mat( height, width, CV_8UC3 ); // FIXME type is hard-coded here
		std::cout << sizeof(&(img.data)) << std::endl;
		boost::asio::read(socket_, boost::asio::buffer(const_cast< char* >( reinterpret_cast< const char* >( img.datastart ) ), sizeof(img.data)) );
		
		// Display image
		std::cout << img.cols << "x" << img.rows << std::endl;
        cv::imshow("Image", img);
        cv::waitKey(5);

		ImageData imageData;
		imageData.timestamp = timestamp;
		imageData.width = width;
		imageData.height = height;
		imageData.type = type;

		return imageData;
	}
*/

	// main read thread
	void read() {
		
		try {
			if (format_=="t,2uw,2ui,uw,3d,3uw,3d") { // lidar
				//std::cerr << "Lidar data." << std::endl;	
				//while(true)
				{
					LidarPoint lidar_point;
					boost::asio::read(socket_, boost::asio::buffer(&lidar_point.timestamp, sizeof(lidar_point.timestamp)));
					boost::asio::read(socket_, boost::asio::buffer(&lidar_point.measurement_id, sizeof(lidar_point.measurement_id)));
					boost::asio::read(socket_, boost::asio::buffer(&lidar_point.frame_id, sizeof(lidar_point.frame_id)));
		    		boost::asio::read(socket_, boost::asio::buffer(&lidar_point.encoder_count, sizeof(lidar_point.encoder_count)));
		    		boost::asio::read(socket_, boost::asio::buffer(&lidar_point.block, sizeof(lidar_point.block)));
					boost::asio::read(socket_, boost::asio::buffer(&lidar_point.channel, sizeof(lidar_point.channel)));
					boost::asio::read(socket_, boost::asio::buffer(&lidar_point.range, sizeof(lidar_point.range)));
					boost::asio::read(socket_, boost::asio::buffer(&lidar_point.bearing, sizeof(lidar_point.bearing)));
					boost::asio::read(socket_, boost::asio::buffer(&lidar_point.elevation, sizeof(lidar_point.elevation)));
					boost::asio::read(socket_, boost::asio::buffer(&lidar_point.signal, sizeof(lidar_point.signal)));
					boost::asio::read(socket_, boost::asio::buffer(&lidar_point.reflectivity, sizeof(lidar_point.reflectivity)));
					boost::asio::read(socket_, boost::asio::buffer(&lidar_point.ambient, sizeof(lidar_point.ambient)));
					boost::asio::read(socket_, boost::asio::buffer(&lidar_point.point[0], sizeof(lidar_point.point)));

					/*std::cout << " ts " << lidar_point.timestamp
							<< " mid " << lidar_point.measurement_id << " fid " << lidar_point.frame_id 
							<< " en " << lidar_point.encoder_count << " bk " << lidar_point.block 
							<< " ch " << lidar_point.channel
							<< " rg " << lidar_point.range << " br " << lidar_point.bearing << " el " << lidar_point.elevation
							<< " sg " << lidar_point.signal << " rf " << lidar_point.reflectivity << " am " << lidar_point.ambient
							<< " pt " << lidar_point.point[0] << "," << lidar_point.point[1] << "," << lidar_point.point[2] << std::endl;*/

					if (currentFrameId!=lidar_point.frame_id) { // check if this is a new scan // FIXME seems like block is more stable to use then frame_id
						if (currentFrameId!=0) std::cout << "Number of points in scan " << currentFrameId << " = " << lidar_scan.size() << std::endl;
						lidar_scan.clear();
						lidar_scan.resize(0);
						currentFrameId = lidar_point.frame_id;
					}
					// FIXME filter points before adding them, which field to use?
					if (lidar_point.reflectivity!=0)
						lidar_scan.push_back(lidar_point);

				}
					
				// FIXME need to insert lidar data into a queue
			}
			else if (format_=="t,3ui,s[15197952]") { // camera
				//while(true)
				{
					boost::asio::streambuf receiveBuffer;
					//std::cerr << "Image data." << std::endl;
					readHeader(socket_, receiveBuffer, IMAGE_HEADER_SIZE);
					readPixelData(socket_, receiveBuffer, IMAGE_SIZE);
					//std::cout << "Buffer size " << receiveBuffer.size() << std::endl;
					//Process the header and read the pixel data
					std::pair< boost::posix_time::ptime, cv::Mat > p = processReceivedData(receiveBuffer);
					// Handle the received image data
					handleImageData(p);
					// FIXME need to insert image data into a queue
				}
			}
			else
				std::cerr << "Unknown input format : " << format_ << std::endl;
		} catch (const std::exception& ex) {
        	std::cerr << "Read Error: " << ex.what() << std::endl;
    	}
	}

private:
	tcp::resolver resolver_;
	tcp::socket socket_;
	std::string port_, format_;
	size_t size_;
	mutable boost::function< void() > close_;

	LidarScan lidar_scan;
	std::uint16_t currentFrameId = 0;
};

// this class manages multiple tcp clients
class tcp_client
{
public:
    tcp_client( boost::asio::io_service& service, const std::string& server, const std::vector<std::pair<std::string, std::string>>& ports)
		: io_service_(service), server_(server), ports_(ports){}

	// construct and connect clients
	void connect_all() {
		for (auto& port : ports_) { // FIXME const auto& port
			connections_.emplace_back(std::make_unique<connection>(io_service_, server_, port)); // FIXME make_unique? emplace_back?
		}
	}

	// read from clients
	void read_all() {
		while(true)
		{
			for (auto& connection : connections_) {
				connection->read();
			}
		}
	}

private:
	boost::asio::io_service& io_service_; // FIXME why &
	std::string server_;
	std::vector<std::pair<std::string, std::string>> ports_;
	std::vector<std::unique_ptr<connection>> connections_; 

};

int main(int argc, char* argv[])
{
    try
    {
		/*if (argc !=3) {
			std::cerr << "Usage: " << argv[0] << " <host> <port>" << std::endl;
			return 1;
		}*/

		comma::command_line_options options( argc, argv, usage );
		std::vector< std::string > inputs = options.unnamed( "--help,-h,--verbose,-v", "-.*" );

		if( inputs.size() < 1 )
		{
			std::cerr << comma::verbose.app_name() << ": requires <list-of-ports>" << std::endl;
        	return 1;
		}

		std::vector<std::pair<std::string, std::string>> ports;
        for( const std::string& input : inputs ) 
		{ 
			std::vector< std::string > input_components = comma::split( input, ';' );
        	std::string port = input_components[0];
        	std::string format = comma::split( input_components[1], '=' )[1];
			ports.push_back(std::make_pair(port, format));
		}

		boost::asio::io_service io_service;
		//std::vector<std::string> ports = {"<port_number1>", "<port_number2>"}; // replace with actual port numbers
		tcp_client client(io_service, "localhost", ports); // replace with actual IP address

		client.connect_all();
		client.read_all();
	}
    catch (std::exception& e)
    {
        std::cerr << "[" << argv[0] << "] Exception: " << e.what() << std::endl;
    }
	catch ( ... )
	{
		std::cerr << "[" << argv[0] << "] unknown exception" << std::endl;
	}

    return 0;
	
}