#include <boost/format.hpp>
#include "utils.hpp"

#ifndef SEMANTIC_KITTI_LOADER_HPP
#define SEMANTIC_KITTI_LOADER_HPP

template<typename T>
class SemanticKittiLoader {
public:
    SemanticKittiLoader(const std::string &abs_path) {
        std::cout << "Load path : " << abs_path << std::endl;
        pc_path_ = abs_path + "/velodyne";
        label_path_ = abs_path + "/labels";

        for (num_frames_ = 0;; num_frames_++) {
            std::string filename = (boost::format("%s/%06d.bin") % pc_path_ % num_frames_).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }
        int num_labels;
        for (num_labels = 0;; num_labels++) {
            std::string filename = (boost::format("%s/%06d.label") % label_path_ % num_labels).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }

        if (num_frames_ == 0) {
            std::cerr << "Error: no files in " << pc_path_ << std::endl;
        }
        if (num_frames_ != num_labels) {
            std::cerr << "Error: The # of point clouds and # of labels are not same" << std::endl;
        }
    }

    ~SemanticKittiLoader() {}

    size_t size() const { return num_frames_; }

    int get_cloud(size_t idx, pcl::PointCloud<T> &cloud) const {
        std::string filename = (boost::format("%s/%06d.bin") % pc_path_ % idx).str();
        FILE *file = fopen(filename.c_str(), "rb");
        if (!file) {
            std::cerr << "error: failed to load " << filename << std::endl;
            return -1;
        }

        std::vector<float> buffer(1000000);
        size_t num_points = fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
        fclose(file);

        cloud.resize(num_points);
        if (std::is_same<T, pcl::PointXYZ>::value) {
            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
            }
        } else if (std::is_same<T, pcl::PointXYZI>::value) {
            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
                pt.intensity = buffer[i * 4 + 3];
            }

        } else if (std::is_same<T, PointXYZILID>::value) {
            std::string label_name = (boost::format("%s/%06d.label") % label_path_ % idx).str();
            std::ifstream label_input(label_name, std::ios::binary);
            if (!label_input.is_open()) {
                std::cerr << "Could not open the label!" << std::endl;
                return -1;
            }
            label_input.seekg(0, std::ios::beg);

            std::vector<uint32_t> labels(num_points);
            label_input.read((char*)&labels[0], num_points * sizeof(uint32_t));

            for (int i = 0; i < num_points; i++) {
                auto &pt = cloud.at(i);
                pt.x = buffer[i * 4];
                pt.y = buffer[i * 4 + 1];
                pt.z = buffer[i * 4 + 2];
                pt.intensity = buffer[i * 4 + 3];
                pt.label = labels[i] & 0xFFFF;
                pt.id = labels[i] >> 16;
            }

            label_input.close();
        }

        return num_points;
    }

private:
    int num_frames_;
    std::string label_path_;
    std::string pc_path_;
};

#endif
