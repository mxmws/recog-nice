// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <algorithm>            // std::min, std::max
#include <cmath>
#include <chrono>
#include <thread>

int main(int argc, char * argv[]) try
{

    rs2::context ctx;
    rs2::device_list list = ctx.query_devices();
    std::cout << list.size() << " devices found" << std::endl;
    
    if(0 >= (int)list.size())
    {
        std::cout << "Connect Realsense first! Exit program." <<std::endl;
        return -1;
    }
    
//    std::cout << "Reset device... " << std::endl;
//    rs2::device dev = list.front();
//    rs2::device_hub hub(ctx);
//
//    dev.hardware_reset();
//    std::this_thread::sleep_for( std::chrono::seconds(1));
//    std::cout << "...done!" << std::endl;
//    
//    std::cout << "Wait for reconnecting device..." << std::endl;
//    while( !hub.is_connected(dev))
//    {
//        std::this_thread::sleep_for( std::chrono::milliseconds(100));
//    }
//    std::cout << "...done!" << std::endl;
    
    // Declare filters
    rs2::decimation_filter dec_filter;
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 4);
    
    rs2::threshold_filter thresh_filter;
    thresh_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.15f);
    thresh_filter.set_option(RS2_OPTION_MAX_DISTANCE, 4.00f);
    
    rs2::spatial_filter spat_filter;
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.50f);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0f);
    spat_filter.set_option(RS2_OPTION_HOLES_FILL, 0.0f);
    
    rs2::temporal_filter temp_filter;
    temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3.0f);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
    
    rs2::disparity_transform depth2disparity;
    
    rs2::disparity_transform disparity2depth(false);
    

    
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
     // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    
    
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_RGB8, 30);
    //cfg.disable_stream(RS2_STREAM_COLOR );
    
    // Start streaming with default recommended configuration
    rs2::pipeline_profile profile = pipe.start(cfg);

    rs2::depth_sensor depth_sensor = profile.get_device().first<rs2::depth_sensor>();
    //depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY); //führt oft zum crash!!!
    double depth_scale = depth_sensor.get_depth_scale();
    double exposuretime = depth_sensor.get_option(RS2_OPTION_EXPOSURE);
    double gain = depth_sensor.get_option(RS2_OPTION_GAIN);
    double laserpower = depth_sensor.get_option(RS2_OPTION_LASER_POWER);
    

    std::cout << "Check params..." << std::endl;
    
    std::cout << " Depth_Option_Units: " << depth_scale << std::endl;
    if(fabs(0.0001 -depth_scale) > 0.0001)
    {
        depth_scale = 0.0001;
        std::cout << " set Depth Scale to " << depth_scale;
        depth_sensor.set_option(RS2_OPTION_DEPTH_UNITS, depth_scale);
        depth_scale = depth_sensor.get_depth_scale();
        std::cout << "; New Depth_Option_Units: " << depth_scale << std::endl;        
    }
    
    std::cout << " Exposure Time: " << exposuretime << "us" << std::endl;
    if(fabs(60000.0 - exposuretime) > 0.0001)
    {
        double val = 60000.0;
        std::string name("Exposure Time");
        std::cout << " set " << name <<" to " << val << " us";
        depth_sensor.set_option(RS2_OPTION_EXPOSURE, val);
        val = depth_sensor.get_option(RS2_OPTION_EXPOSURE);
        std::cout << "; New " << name << ": " << val << " us" << std::endl;        
    }
    
    std::cout << " Gain: " << gain <<  std::endl;
    if(fabs(16.0 - gain) > 0.0001)
    {
        double val = 16.0;
        std::string name("Gain");
        std::cout << " set "<< name << " to " << val;
        depth_sensor.set_option(RS2_OPTION_GAIN, val);
        val = depth_sensor.get_option(RS2_OPTION_GAIN);
        std::cout << "; New "<< name << ": " << val << " us" << std::endl;        
    }
    
    std::cout << " Laser power: " << laserpower <<  std::endl;
    if(fabs(150.0 - laserpower) > 0.0001)
    {
        double val = 150.0;
        std::string name("Laser Power");
        std::cout << " set "<< name << " to " << val;
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, val);
        val = depth_sensor.get_option(RS2_OPTION_LASER_POWER);
        std::cout << "; New "<< name << ": " << val << " us" << std::endl;        
    }
    
    //std::cout << " "
    
    
    int i=0;
    const int iMax = 15;
    
    std::cout << "Start" << std::endl;

    while(i<iMax)
    {
        i++;
         std::cout << "Run: " << i << std::endl;
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames(); 
        
        rs2::frame depth = frames.get_depth_frame();
        
        depth = dec_filter.process(depth);
        depth = depth2disparity.process(depth);
        depth = spat_filter.process(depth);
        depth = temp_filter.process(depth);
        depth = disparity2depth.process(depth);

        if((iMax)==i)
        {
            pipe.stop();
            
            rs2::video_frame vframe = frames.get_color_frame();
            points = pc.calculate(depth);
            
            std::string sfilename("test1_filter15.ply");
            // Generate the pointcloud and texture mappings
        
            points.export_to_ply(sfilename, vframe);
            std::cout << sfilename << " has been written" << std::endl;
            std::cout << "hello my name is maximilian" << std::endl;
        }
    }   

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
