// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../../../examples/example.hpp" // Include short list of convenience functions for rendering

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, 1);
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

int main(int argc, char * argv[])
{
    rs2::context ctx;// Create librealsense context for managing devices
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    std::vector<rs2::pipeline> pipelines;

    //Add desired streams to configuration
    const int RGB_STREAM_WIDTH = 640;//1280;
    const int RGB_STREAM_HEIGHT = 480;//720;
    const int FRAME_RATE = 30;
    const int DEPTH_STREAM_WIDTH = 848;
    const int DEPTH_STREAM_HEIGHT = 480;
    
    // Start a streaming pipe per each connected device

    for (auto&& dev : ctx.query_devices())
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        //uncomment this will rise a `failed to set power state`bug
        //cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        std::cout<<"serial num = "<<dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)<<std::endl;

        cfg.enable_stream(RS2_STREAM_COLOR, RGB_STREAM_WIDTH, RGB_STREAM_HEIGHT, RS2_FORMAT_BGR8, FRAME_RATE);
        cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_STREAM_WIDTH, DEPTH_STREAM_HEIGHT, RS2_FORMAT_Z16, FRAME_RATE);

        //cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        //cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        // Start streaming with default recommended configuration
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
    }

//    auto devs = ctx.query_devices();
//    rs2::pipeline pipe1(ctx);
//    rs2::config cfg1;
//    //uncomment this will rise a `failed to set power state`bug
//    cfg1.enable_device(devs[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
//    std::cout<<"serial num = "<<devs[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)<<std::endl;

//    cfg1.enable_stream(RS2_STREAM_COLOR, RGB_STREAM_WIDTH, RGB_STREAM_HEIGHT, RS2_FORMAT_BGR8, FRAME_RATE);
//    cfg1.enable_stream(RS2_STREAM_DEPTH, DEPTH_STREAM_WIDTH, DEPTH_STREAM_HEIGHT, RS2_FORMAT_Z16, FRAME_RATE);
//    // Start streaming with default recommended configuration

//    pipelines.emplace_back(pipe1);

//    rs2::pipeline pipe2(ctx);
//    rs2::config cfg2;
//    //uncomment this will rise a `failed to set power state`bug
//    cfg2.enable_device(devs[1].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
//    std::cout<<"serial num = "<<devs[1].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)<<std::endl;

//    cfg2.enable_stream(RS2_STREAM_COLOR, RGB_STREAM_WIDTH, RGB_STREAM_HEIGHT, RS2_FORMAT_BGR8, FRAME_RATE);
//    cfg2.enable_stream(RS2_STREAM_DEPTH, DEPTH_STREAM_WIDTH, DEPTH_STREAM_HEIGHT, RS2_FORMAT_Z16, FRAME_RATE);
//    // Start streaming with default recommended configuration
//    pipelines.emplace_back(pipe2);
//    pipe2.start(cfg2);
//    pipe1.start(cfg1);


    std::cout<<"start"<<std::endl;
    // Define two align objects. One will be used to align
    // to depth viewport and the other to color.
    // Creating align object is an expensive operation
    // that should not be performed in the main loop
    rs2::align align_to_color(RS2_STREAM_COLOR);

    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "Generating example point clouds.\n\n";

    basic_cloud_ptr->width = RGB_STREAM_WIDTH*RGB_STREAM_HEIGHT;
    basic_cloud_ptr->height = 1;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;// (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer = simpleVis(basic_cloud_ptr);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ())
    {
        // Wait for the next set of frames from the camera
        std::vector<rs2::frameset> vfs;
        try {
            for (auto &&pipe : pipelines)
            {
                rs2::frameset fs_t;
                if (pipe.poll_for_frames(&fs_t))
                {
                    vfs.emplace_back(fs_t);
                }
            }

            // Convert the newly-arrived frames to render-friendly format
            for (const auto& fs : vfs)
            {
                // Get the serial number of the current frame's device
                auto serial = rs2::sensor_from_frame(fs)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
 
                auto align2c_frames = align_to_color.process(fs);

                auto align2c_color = align2c_frames.get_color_frame();
                auto align2c_depth = align2c_frames.get_depth_frame();
                // Generate the pointcloud and texture mappings
                points = pc.calculate(align2c_depth);
                
                cv::Mat align2c_Color(cv::Size(align2c_color.get_width(), align2c_color.get_height()), CV_8UC3, (void*)align2c_color.get_data(), cv::Mat::AUTO_STEP);
                cv::Mat align2c_Depth(cv::Size(align2c_depth.get_width(), align2c_depth.get_height()), CV_16U, (void*)align2c_depth.get_data(), cv::Mat::AUTO_STEP);

                if( std::string(serial) == "XXXXXXXXXXXX"){//
//                    cv::imshow("test", align2c_Color);
//                    cv::waitKey(1);
                }
                else if( std::string(serial) == "XXXXXXXXXXXX"){//
                    basic_cloud_ptr->clear();
                    // We're going to make an ellipse extruded along the z-axis.
                    auto vertices = rs_points.get_vertices();// get vertices
                    for (int i = 0; i < rs_points.size(); i++)
                    {
			            if (!isnanf(vertices[i].z)
			                && (vertices[i].z < 0.8)
			                && (vertices[i].z > 0.005)
			                //&& (-vertices[i].y > -0.005)
			                && (vertices[i].y > 0.14))
			            {
			                basic_cloud_ptr->points.emplace_back( vertices[i].z, -vertices[i].x, -vertices[i].y );
			            }
        			}

//                    for ( int _row = 0; _row < align2c_Color.rows; _row++ )
//                    {
//                        for ( int _col = 0; _col < align2c_Color.cols; _col++ )
//                        {
//                            unsigned int d = align2c_Depth.ptr<unsigned short>(_row)[_col]; //
//                            float x, y, z;
//                            z = float(d) * depthScale;
//                            x = (_col - cx) * z / fx;
//                            y = (_row - cy) * z / fy;
//                            pcl::PointXYZ basic_point(x, y, z);
//                            basic_cloud_ptr->points.push_back( basic_point );
//                        }
//                    }

                    basic_cloud_ptr->is_dense = true;
                    boost::mutex::scoped_lock updateLock(updateModelMutex);
                    viewer->updatePointCloud<pcl::PointXYZ>(basic_cloud_ptr,"sample cloud");
                    updateLock.unlock();
                    viewer->spinOnce (100);
                }
            }

        }

        catch (const rs2::error & e)
        {
            std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
            continue;
        }

    }
   
    // close all thread
    std::cout<<"over  "<<std::endl;
    return 1;
}
