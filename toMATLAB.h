#pragma once
#include <fstream>
#include <iostream>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <Eigen/Core>

#include <boost/serialization/vector.hpp>

#include <pcl/common/common.h>
using namespace std;

namespace boost { namespace serialization {

template<   class Archive,
            class S,
            int Rows_,
            int Cols_,
            int Ops_,
            int MaxRows_,
            int MaxCols_>
inline void serialize(Archive & ar,
                      Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_> & matrix,
                      const unsigned int version)
{
    int rows = matrix.rows();
    int cols = matrix.cols();
    ar & make_nvp("rows", rows);
    ar & make_nvp("cols", cols);
    matrix.resize(rows, cols); // no-op if size does not change!

    // always save/load row-major
    for(int r = 0; r < rows; ++r)
        for(int c = 0; c < cols; ++c)
            ar & make_nvp("val", matrix(r,c));
}

template<   class Archive,
            class S,
            int Dim_,
            int Mode_,
            int Options_>
inline void serialize(Archive & ar,
                      Eigen::Transform<S, Dim_, Mode_, Options_> & transform,
                      const unsigned int version)
{
    serialize(ar, transform.matrix(), version);
}
}} // namespace boost::serialization

class edge_data
{
public:
    edge_data()
    {
        edgePoint << 0, 0, 0;
        linePoint_1 << 0, 0, 0;
        linePoint_2 << 0, 0, 0;
        point_line_distance = 0;
    }

    // edge points
    Eigen::Vector3f edgePoint;
    Eigen::Vector3f linePoint_1;
    Eigen::Vector3f linePoint_2;
    float point_line_distance;

    friend class boost::serialization::access;
    template<class Archive> void serialize(Archive & ar,
                                           const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(edgePoint);
        ar & BOOST_SERIALIZATION_NVP(linePoint_1);
        ar & BOOST_SERIALIZATION_NVP(linePoint_2);
        ar & BOOST_SERIALIZATION_NVP(point_line_distance);
    }

};

class iter_data
{
public:
    int iterCount;
    std::vector<edge_data> edge_data_vec;
    //std::vector<plane_data>

    friend class boost::serialization::access;
    template<class Archive> void serialize(Archive & ar,
                                           const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(iterCount);
        ar & BOOST_SERIALIZATION_NVP(edge_data_vec);
    }
};

class ObservingData
{
public:
    std::vector<iter_data> iter_data_vec;
    friend class boost::serialization::access;
    template<class Archive> void serialize(Archive & ar,
                                           const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(iter_data_vec);
    }
};

//class Test {
//private:
//    friend class boost::serialization::access;
//    template<class Archive> void serialize(Archive & ar,
//            const unsigned int version) {
//        ar & BOOST_SERIALIZATION_NVP(a);
//        ar & BOOST_SERIALIZATION_NVP(b);
//        ar & BOOST_SERIALIZATION_NVP(c);
//    }

//    int a;
//    int b;
//    float c;
//public:
//    inline Test(int a, int b, float c) {
//        this->a = a;
//        this->b = b;
//        this->c = c;
//    }
//};

class toMATLAB
{
public:
    void write()
    {
        std::ofstream ofs("filename.xml");

        ObservingData* test = new ObservingData();
        iter_data temp;
        temp.iterCount = 14;
        edge_data temp2;
        temp.edge_data_vec.push_back(temp2);
        temp.edge_data_vec.push_back(temp2);
        test->iter_data_vec.push_back(temp);
        test->iter_data_vec.push_back(temp);
        test->iter_data_vec.push_back(temp);

        boost::archive::xml_oarchive oa(ofs);
        oa << BOOST_SERIALIZATION_NVP(test);
    }

    void writePCLToCSV(std::string name, pcl::PointCloud<pcl::PointXYZ>::Ptr input)
    {
        std::stringstream filename;
        filename << name << ".csv";
        std::string file = filename.str();

        std::ofstream myfile;
        myfile.open (file);

        for (int i=0;i<input->points.size();i++)
        {
            myfile << input->points[i].x << input->points[i].y << input->points[i].z << "\n";
        }

        myfile.close();
    }
};

//int main() {
//    std::ofstream ofs("filename.xml");

//    Test* test = new Test(1, 2, 3.3);

//    boost::archive::xml_oarchive oa(ofs);
//    oa << BOOST_SERIALIZATION_NVP(test);

//    return 0;
//}
