#pragma once

#include <cstdint>

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

//SERIALIZATION
#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/concepts/pair_associative_container.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/access.hpp>

static std::size_t channels = 3;

struct ImageRecord{  // t,3ui,15197952ub
	boost::posix_time::ptime time_stamp;
    std::uint32_t block;
    std::size_t rows;
    std::size_t cols;
    std::uint32_t type;
	void* buffer; // FIXME does this need to be a buffer?

	std::size_t size = ::channels * sizeof( T );
    static std::vector< char > buffer;
    buffer.resize( size );

    template<class Archive> auto serialize( Archive & archive )->void {
        archive(time_stamp, rows, cols, type, buffer);
    }
};