// Copyright (c) 2023 Mission Systems Pty Ltd

#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/io/stream.h>
#include <comma/visiting/traits.h>
#include <boost/date_time/posix_time/posix_time.hpp>

void usage( bool )
{
    std::cerr << "\nSample application for reading binary data streams";
    std::cerr << "\n";
    std::cerr << "\nMakes some major assumptions about the input data";
    std::cerr << "\n";
    std::cerr << "\nUsage: sample-reader /path/to/data";
    std::cerr << "\n";
    std::cerr << "\nOptions:";
    std::cerr << "\n    --help,-h:     display this help message and exit";
    std::cerr << "\n    --verbose,-v:  more output";
    std::cerr << "\n";
    std::cerr << "\nExample:";
    std::cerr << "\n    sample-reader format_testing";
    std::cerr << "\n" << std::endl;
}

struct timestamped_data
{
    boost::optional<boost::posix_time::ptime> timestamp;
    timestamped_data() {}
    timestamped_data( const boost::posix_time::ptime& timestamp ) : timestamp( timestamp ) {}
};

namespace comma { namespace visiting {

template <> struct traits< timestamped_data >
{
    template < typename K, typename V > static void visit( const K&, const timestamped_data& p, V& v ) { v.apply( "t", p.timestamp ); }
    template < typename K, typename V > static void visit( const K&, timestamped_data& p, V& v ) { v.apply( "t", p.timestamp ); }
};

} } // namespace comma { namespace visiting {

int process( const std::string& data_dir )
{
    std::string lidar_data_dir = data_dir + "/ouster/cooked/20230424T060913.969605.bin";
    std::string camera_data_dir = data_dir + "/cameras/alvium_1800_out/20230424T060914.204749.bin";

    comma::io::istream lidar_istream( lidar_data_dir, comma::io::mode::binary );
    comma::csv::binary_input_stream< timestamped_data > lidar_stream( *lidar_istream, "t,2uw,2ui,uw,3d,3uw,3d" );

    comma::io::istream camera_istream( camera_data_dir, comma::io::mode::binary );
    comma::csv::binary_input_stream< timestamped_data > camera_stream( *camera_istream, "t,3ui,s[15197952]" );

    comma::io::select select;
    select.read().add( lidar_istream.fd() );
    select.read().add( camera_istream.fd() );

    comma::signal_flag is_shutdown;
    bool end_of_input = false;
    const timestamped_data* lidar_record = nullptr;
    const timestamped_data* camera_record = nullptr;

    while( !is_shutdown && !end_of_input )
    {
        if( select.check() )
        {
            if( select.read().ready( lidar_istream.fd() ))
            {
                lidar_record = lidar_stream.read();
                if( lidar_record )
                {
                    std::cerr << "got lidar record of " << lidar_stream.size() << " bytes" << std::endl;
                }
            }
            if( select.read().ready( camera_istream.fd() ))
            {
                camera_record = camera_stream.read();
                if( camera_record )
                {
                    std::cerr << "got camera record of " << camera_stream.size() << " bytes" << std::endl;
                }
            }
        }
    }
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        std::vector< std::string > unnamed = options.unnamed( "--help,-h,--verbose,-v", "-.*" );
        if( unnamed.size() == 1 )
        {
            std::string data_dir = unnamed[0];
            return process( data_dir );
        }
        std::cerr << comma::verbose.app_name() << ": requires <path-to-data>" << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 1;
}
