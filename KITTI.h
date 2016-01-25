/**
* This file is part of KITTI_util.
*
* Copyright 2016 Sebastian Dingler <s dot dingler at gmail dot com>
*
* KITTI_util is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* KITTI_util is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with KITTI_util. If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once
#include <iostream>
#include <vector>
#include <dirent.h>
#include <sstream>
#include <fstream>
#include <algorithm>

#include <pcl/point_cloud.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/organized.h>

#include <pcl/filters/passthrough.h>

#include <pcl/common/common.h>

#include <opencv2/opencv.hpp>

#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "tf2_msgs/TFMessage.h"

struct veloPoint{
    float x;
    float y;
    float z;
    float i;
};

struct errors {
    int32_t first_frame;
    float   r_err;
    float   t_err;
    float   len;
    float   speed;
    errors (int32_t first_frame,float r_err,float t_err,float len,float speed) :
        first_frame(first_frame),r_err(r_err),t_err(t_err),len(len),speed(speed) {}
};

//float lengths[] = {100,200,300,400,500,600,700,800};
float lengths[] = {5,10,50,100,150,200,250,300,350,400};
int32_t num_lengths = 8;

class KITTI
{
public:
    KITTI(int sequence, int max, int startOffset);

    // methods for velodyne data
    int getVel(std::vector<std::string> &files, std::vector<std::vector<veloPoint> > &points, int num, int start);
    void getPointCloud2(sensor_msgs::PointCloud2 & outPC, int i);
    void getPointCloud(pcl::PointCloud<pcl::PointXYZ> & msg, int i);
    void getVeloPC(pcl::PointCloud<pcl::PointXYZ> & msg, std::vector<veloPoint> & veloPoints);
    int getOneVel(std::vector<veloPoint> &points, int j);
    int getOneVel(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, int i);

    int getGtCameraPoses(std::vector<Eigen::Matrix3d> &Rs, std::vector<Eigen::Vector3d> &ts);
    void getGtCameraPoses(std::vector<Eigen::Matrix4d> &Ts);
    void getGtCameraPosesAsNavMsg(std::vector<nav_msgs::Odometry> &out);

    void writeResult(std::vector<Eigen::Matrix4d> Ts);


    Eigen::Matrix3f getK();
    cv::Mat getImage_0(int i);
    int getWidth();
    int getHeight();
    Eigen::Matrix3Xd getP0();
    Eigen::Matrix4d get_T_velo_to_cam();
    Eigen::Matrix4d getVelo_to_cam_T();
    int size() {return seq_size;}
    Eigen::Matrix4d getGroundTruthByID(int id)
    {
        int idx = id-startOffset;
        return gt_T[idx];
    }
    Eigen::Matrix4d getGroundTruthDeltaByID(int id)
    {
        int idx = id-startOffset;
        if (idx>0)
            return poseDelta(gt_T[idx-1], gt_T[idx]);
        else
            return Eigen::Matrix4d::Identity();
    }
    Eigen::Matrix4d computeError(std::vector<Eigen::Matrix4d> & T_result)
    {
        Eigen::Matrix4d delta_T_result = poseDelta(T_result[0],T_result.back());
        Eigen::Matrix4d delta_T_gt = poseDelta(gt_T[0],gt_T.back());
        return poseDelta(delta_T_result,delta_T_gt);
    }
    Eigen::Matrix4d computeError(std::vector<Eigen::Matrix4d> & T_first,std::vector<Eigen::Matrix4d> & T_second)
    {
        Eigen::Matrix4d delta_T_result = poseDelta(T_first[0],T_first.back());
        Eigen::Matrix4d delta_T_gt = poseDelta(T_second[0],T_second.back());
        return poseDelta(delta_T_result,delta_T_gt);
    }
    bool eval(std::vector<Eigen::Matrix4d> & T_result);
    void rotToOpencv(Eigen::Matrix4d &in)
    {
        Eigen::Matrix4d rotToOpenCV = Eigen::Matrix4d::Identity();
        rotToOpenCV(0,0) = -1;
        rotToOpenCV(1,1) = -1;
        rotToOpenCV(2,2) = 1;
        rotToOpenCV(0,1) = rotToOpenCV(0,2) = rotToOpenCV(0,3) = rotToOpenCV(1,0) = rotToOpenCV(1,2) = rotToOpenCV(1,3) = rotToOpenCV(2,0) = rotToOpenCV(2,1) = rotToOpenCV(2,3) = 0;
        in = in * rotToOpenCV;
    }
    void plotRainbow(cv::Mat & image, std::vector<veloPoint> & velpoints,  Eigen::Matrix4d &T, Eigen::Matrix3Xd & P0);

    void dispLidarInImage(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc, int id);
    void dispLidarInImage(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, int id);
    Eigen::Matrix4d poseDelta(Eigen::Matrix4d &T_first, Eigen::Matrix4d &T_last)
    {
        Eigen::Matrix4d T_delta = T_first.inverse()*T_last;
        return T_delta;
    }
private:
    int sequence;
    int startOffset;
    void createPointCloud2(sensor_msgs::PointCloud2 & outPC, std::vector<veloPoint> & veloPoints);
    Eigen::Matrix3d getVelo_to_cam_R();
    Eigen::Vector3d getVelo_to_cam_t();
    std::vector<std::string> image_0_files;
    int width;
    int height;
    std::string path_to_image_0;
    std::string path_to_image_2;
    std::string path_to_velo;
    std::string pathPoses;
    std::string result_dir;

    std::vector<std::string> velo_files;
    std::vector<std::vector<veloPoint>> velpoints;

    int getFiles(std::string source, std::vector<std::string> &files);
    int getdir(std::string dir, std::vector<std::string> &files);
    int getFile (std::string source, std::vector<std::string> &files);
    int seq_size;
    std::vector<Eigen::Matrix4d> gt_T;
    inline double rotationError(Eigen::Matrix4d &T) {
        double a = T(0,0);
        double b = T(1,1);
        double c = T(2,2);
        double d = 0.5*(a+b+c-1.0);
        return acos(std::max(std::min(d,1.0),-1.0));
    }

    inline double translationError(Eigen::Matrix4d &T) {
        float dx = T(0,3);
        float dy = T(1,3);
        float dz = T(2,3);
        return sqrt(dx*dx+dy*dy+dz*dz);
    }



    std::vector<errors> calcSequenceErrors (std::vector<Eigen::Matrix4d> &T_result,std::vector<Eigen::Matrix4d> &T_gt) {

        // error vector
        std::vector<errors> err;

        // parameters
        int32_t step_size = 10; // every second

        // pre-compute distances (from ground truth as reference)
        std::vector<float> dist = trajectoryDistances(T_gt);

        // for all start positions do
        for (int first_frame=0; first_frame<T_gt.size(); first_frame+=step_size)
        {

            // for all segment lengths do
            for (int32_t i=0; i<num_lengths; i++)
            {

                // current length
                float len = lengths[i];

                // compute last frame
                int32_t last_frame = lastFrameFromSegmentLength(dist,first_frame,len);

                // continue, if sequence not long enough
                if (last_frame==-1)
                    continue;

                // compute rotational and translational errors
                Eigen::Matrix4d T_error = computeError(T_result);
                double r_err = rotationError(T_error);
                double t_err = translationError(T_error);

                // compute speed
                float num_frames = (float)(last_frame-first_frame+1);
                float speed = len/(0.1*num_frames);

                // write to file
                err.push_back(errors(first_frame,r_err/len,t_err/len,len,speed));
            }
        }

        // return error vector
        return err;
    }

    std::vector<float> trajectoryDistances (std::vector<Eigen::Matrix4d> &poses) {
        std::vector<float> dist;
        dist.push_back(0);
        for (int32_t i=1; i<poses.size(); i++) {
            Eigen::Matrix4d P1 = poses[i-1];
            Eigen::Matrix4d P2 = poses[i];
            float dx = P1(0,3)-P2(0,3);
            float dy = P1(1,3)-P2(1,3);
            float dz = P1(2,3)-P2(2,3);
            dist.push_back(dist[i-1]+sqrt(dx*dx+dy*dy+dz*dz));
        }
        return dist;
    }

    int32_t lastFrameFromSegmentLength(std::vector<float> &dist,int32_t first_frame,float len) {
        for (int32_t i=first_frame; i<dist.size(); i++)
            if (dist[i]>dist[first_frame]+len)
                return i;
        return -1;
    }

    std::vector<int32_t> computeRoi (std::vector<Eigen::Matrix4d> &poses_gt,std::vector<Eigen::Matrix4d> &poses_result) {

        double x_min = std::numeric_limits<int32_t>::max();
        double x_max = std::numeric_limits<int32_t>::min();
        double z_min = std::numeric_limits<int32_t>::max();
        double z_max = std::numeric_limits<int32_t>::min();

        for (int i=0; i< poses_gt.size(); i++) {
            double x = poses_gt[i](0,3);
            double z = poses_gt[i](2,3);
            if (x<x_min) x_min = x; if (x>x_max) x_max = x;
            if (z<z_min) z_min = z; if (z>z_max) z_max = z;
        }

        for (int i=0; i< poses_result.size(); i++) {
            double x = poses_result[i](0,3);
            double z = poses_result[i](2,3);
            if (x<x_min) x_min = x; if (x>x_max) x_max = x;
            if (z<z_min) z_min = z; if (z>z_max) z_max = z;
        }

        double dx = 1.1*(x_max-x_min);
        double dz = 1.1*(z_max-z_min);
        double mx = 0.5*(x_max+x_min);
        double mz = 0.5*(z_max+z_min);
        double r  = 0.5*std::max(dx,dz);

        std::vector<int32_t> roi;
        roi.push_back((int32_t)(mx-r));
        roi.push_back((int32_t)(mx+r));
        roi.push_back((int32_t)(mz-r));
        roi.push_back((int32_t)(mz+r));
        return roi;
    }

    void plotPathPlot (std::string dir,std::vector<int32_t> &roi,int32_t idx) {

        // gnuplot file name
        char command[1024];
        char file_name[256];
        std::sprintf(file_name,"%02d.gp",idx);
        std::string full_name = dir + "/" + file_name;

        // create png + eps
        for (int32_t i=0; i<2; i++) {

            // open file
            FILE *fp = fopen(full_name.c_str(),"w");

            // save gnuplot instructions
            if (i==0) {
                fprintf(fp,"set term png size 900,900\n");
                fprintf(fp,"set output \"%02d.png\"\n",idx);
            } else {
                fprintf(fp,"set term postscript eps enhanced color\n");
                fprintf(fp,"set output \"%02d.eps\"\n",idx);
            }

            fprintf(fp,"set size ratio -1\n");
            fprintf(fp,"set xrange [%d:%d]\n",roi[0],roi[1]);
            fprintf(fp,"set yrange [%d:%d]\n",roi[2],roi[3]);
            fprintf(fp,"set xlabel \"x [m]\"\n");
            fprintf(fp,"set ylabel \"z [m]\"\n");
            fprintf(fp,"plot \"%02d.txt\" using 1:2 lc rgb \"#FF0000\" title 'Ground Truth' w lines,",idx);
            fprintf(fp,"\"%02d.txt\" using 3:4 lc rgb \"#0000FF\" title 'Visual Odometry' w lines,",idx);
            fprintf(fp,"\"< head -1 %02d.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' w points\n",idx);

            // close file
            fclose(fp);

            // run gnuplot => create png + eps
            sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
            system(command);
        }

        // create pdf and crop
        sprintf(command,"cd %s; ps2pdf %02d.eps %02d_large.pdf",dir.c_str(),idx,idx);
        system(command);
        sprintf(command,"cd %s; pdfcrop %02d_large.pdf %02d.pdf",dir.c_str(),idx,idx);
        system(command);
        sprintf(command,"cd %s; rm %02d_large.pdf",dir.c_str(),idx);
        system(command);
    }

    void plotErrorPlots (std::string dir,char* prefix) {

        char command[1024];

        // for all four error plots do
        for (int32_t i=0; i<4; i++) {

            // create suffix
            char suffix[16];
            switch (i) {
            case 0: std::sprintf(suffix,"tl"); break;
            case 1: std::sprintf(suffix,"rl"); break;
            case 2: std::sprintf(suffix,"ts"); break;
            case 3: std::sprintf(suffix,"rs"); break;
            }

            // gnuplot file name
            char file_name[1024]; char full_name[1024];
            std::sprintf(file_name,"%s_%s.gp",prefix,suffix);
            std::sprintf(full_name,"%s/%s",dir.c_str(),file_name);

            // create png + eps
            for (int32_t j=0; j<2; j++) {

                // open file
                FILE *fp = fopen(full_name,"w");

                // save gnuplot instructions
                if (j==0) {
                    fprintf(fp,"set term png size 500,250 font \"Helvetica\" 11\n");
                    fprintf(fp,"set output \"%s_%s.png\"\n",prefix,suffix);
                } else {
                    fprintf(fp,"set term postscript eps enhanced color\n");
                    fprintf(fp,"set output \"%s_%s.eps\"\n",prefix,suffix);
                }

                // start plot at 0
                fprintf(fp,"set size ratio 0.5\n");
                fprintf(fp,"set yrange [0:*]\n");

                // x label
                if (i<=1) fprintf(fp,"set xlabel \"Path Length [m]\"\n");
                else      fprintf(fp,"set xlabel \"Speed [km/h]\"\n");

                // y label
                if (i==0 || i==2) fprintf(fp,"set ylabel \"Translation Error [%%]\"\n");
                else              fprintf(fp,"set ylabel \"Rotation Error [deg/m]\"\n");

                // plot error curve
                fprintf(fp,"plot \"%s_%s.txt\" using ",prefix,suffix);
                switch (i) {
                case 0: fprintf(fp,"1:($2*100) title 'Translation Error'"); break;
                case 1: fprintf(fp,"1:($2*57.3) title 'Rotation Error'"); break;
                case 2: fprintf(fp,"($1*3.6):($2*100) title 'Translation Error'"); break;
                case 3: fprintf(fp,"($1*3.6):($2*57.3) title 'Rotation Error'"); break;
                }
                fprintf(fp," lc rgb \"#0000FF\" pt 4 w linespoints\n");

                // close file
                fclose(fp);

                // run gnuplot => create png + eps
                std::sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
                system(command);
            }

            // create pdf and crop
            std::sprintf(command,"cd %s; ps2pdf %s_%s.eps %s_%s_large.pdf",dir.c_str(),prefix,suffix,prefix,suffix);
            system(command);
            std::sprintf(command,"cd %s; pdfcrop %s_%s_large.pdf %s_%s.pdf",dir.c_str(),prefix,suffix,prefix,suffix);
            system(command);
            std::sprintf(command,"cd %s; rm %s_%s_large.pdf",dir.c_str(),prefix,suffix);
            system(command);
        }
    }

    void savePathPlot (std::vector<Eigen::Matrix4d> &poses_gt, std::vector<Eigen::Matrix4d> &poses_result,std::string file_name) {

        // parameters
        int32_t step_size = 3;

        // open file
        FILE *fp = fopen(file_name.c_str(),"w");

        // save x/z coordinates of all frames to file
        for (int32_t i=0; i<poses_gt.size(); i+=step_size)
            fprintf(fp,"%f %f %f %f\n",poses_gt[i](0,3),poses_gt[i](2,3),
                    poses_result[i](0,3),poses_result[i](2,3));

                    // close file
                    fclose(fp);
    }

    void saveErrorPlots(std::vector<errors> &seq_err,std::string plot_error_dir,char* prefix) {

        // file names
        char file_name_tl[1024]; std::sprintf(file_name_tl,"%s/%s_tl.txt",plot_error_dir.c_str(),prefix);
        char file_name_rl[1024]; std::sprintf(file_name_rl,"%s/%s_rl.txt",plot_error_dir.c_str(),prefix);
        char file_name_ts[1024]; std::sprintf(file_name_ts,"%s/%s_ts.txt",plot_error_dir.c_str(),prefix);
        char file_name_rs[1024]; std::sprintf(file_name_rs,"%s/%s_rs.txt",plot_error_dir.c_str(),prefix);

        // open files
        FILE *fp_tl = fopen(file_name_tl,"w");
        FILE *fp_rl = fopen(file_name_rl,"w");
        FILE *fp_ts = fopen(file_name_ts,"w");
        FILE *fp_rs = fopen(file_name_rs,"w");

        // for each segment length do
        for (int32_t i=0; i<num_lengths; i++) {

            float t_err = 0;
            float r_err = 0;
            float num   = 0;

            // for all errors do
            for (std::vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
                if (fabs(it->len-lengths[i])<1.0) {
                    t_err += it->t_err;
                    r_err += it->r_err;
                    num++;
                }
            }

            // we require at least 3 values
            if (num>2.5) {
                fprintf(fp_tl,"%f %f\n",lengths[i],t_err/num);
                fprintf(fp_rl,"%f %f\n",lengths[i],r_err/num);
            }
        }

        // for each driving speed do (in m/s)
        for (float speed=2; speed<25; speed+=2) {

            float t_err = 0;
            float r_err = 0;
            float num   = 0;

            // for all errors do
            for (std::vector<errors>::iterator it=seq_err.begin(); it!=seq_err.end(); it++) {
                if (fabs(it->speed-speed)<2.0) {
                    t_err += it->t_err;
                    r_err += it->r_err;
                    num++;
                }
            }

            // we require at least 3 values
            if (num>2.5) {
                fprintf(fp_ts,"%f %f\n",speed,t_err/num);
                fprintf(fp_rs,"%f %f\n",speed,r_err/num);
            }
        }

        // close files
        fclose(fp_tl);
        fclose(fp_rl);
        fclose(fp_ts);
        fclose(fp_rs);
    }

    void saveStats (std::vector<errors> err,std::string dir) {

        float t_err = 0;
        float r_err = 0;

        // for all errors do => compute sum of t_err, r_err
        for (std::vector<errors>::iterator it=err.begin(); it!=err.end(); it++) {
            t_err += it->t_err;
            r_err += it->r_err;
        }

        // open file
        FILE *fp = fopen((dir + "/stats.txt").c_str(),"w");

        // save errors
        float num = err.size();
        fprintf(fp,"%f %f\n",t_err/num,r_err/num);

        // close file
        fclose(fp);
    }


    cv::Scalar getRainbow(double depth);

};
