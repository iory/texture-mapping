/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Raphael Favier, Technical University Eindhoven, (r.mysurname <aT>
 * tue.nl)
 */

#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <fstream>
#include <iostream>
#include <sstream>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/texture_mapping.h>

using namespace pcl;

std::ifstream& go_to_line(std::ifstream& file, unsigned int num) {
  file.seekg(std::ios::beg);
  for (int i = 0; i < num - 1; ++i) {
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  return (file);
}

bool read_cam_pose_file(std::string filename,
                        pcl::TextureMapping<pcl::PointXYZ>::Camera& cam) {
  ifstream myReadFile;
  myReadFile.open(filename.c_str(), ios::in);
  if (!myReadFile.is_open()) {
    PCL_ERROR("Error opening file %d\n", filename.c_str());
    return false;
  }
  myReadFile.seekg(ios::beg);

  double val;

  go_to_line(myReadFile, 1);
  myReadFile >> val;
  cam.pose(0, 3) = val;  // TX
  myReadFile >> val;
  cam.pose(1, 3) = val;  // TY
  myReadFile >> val;
  cam.pose(2, 3) = val;  // TZ

  go_to_line(myReadFile, 2);
  myReadFile >> val;
  cam.pose(0, 0) = val;
  myReadFile >> val;
  cam.pose(0, 1) = val;
  myReadFile >> val;
  cam.pose(0, 2) = val;

  myReadFile >> val;
  cam.pose(1, 0) = val;
  myReadFile >> val;
  cam.pose(1, 1) = val;
  myReadFile >> val;
  cam.pose(1, 2) = val;

  myReadFile >> val;
  cam.pose(2, 0) = val;
  myReadFile >> val;
  cam.pose(2, 1) = val;
  myReadFile >> val;
  cam.pose(2, 2) = val;

  cam.pose(3, 0) = 0.0;
  cam.pose(3, 1) = 0.0;
  cam.pose(3, 2) = 0.0;
  cam.pose(3, 3) = 1.0;  // Scale

  go_to_line(myReadFile, 5);
  myReadFile >> val;
  cam.focal_length_w = val;
  myReadFile >> val;
  cam.focal_length_h = val;
  myReadFile >> val;
  cam.center_w = val;
  myReadFile >> val;
  cam.center_h = val;
  myReadFile >> val;
  cam.height = val;
  myReadFile >> val;
  cam.width = val;

  // close file
  myReadFile.close();
  return true;
}

int main(int argc, char** argv) {
  pcl::PolygonMesh triangles;
  pcl::io::loadPolygonFilePLY(argv[1], triangles);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(triangles.cloud, *cloud);

  // Create the texturemesh object that will contain our UV-mapped mesh
  TextureMesh mesh;
  mesh.cloud = triangles.cloud;
  std::vector<pcl::Vertices> polygon_1;

  // push faces into the texturemesh object
  polygon_1.resize(triangles.polygons.size());
  for (size_t i = 0; i < triangles.polygons.size(); ++i) {
    polygon_1[i] = triangles.polygons[i];
  }
  mesh.tex_polygons.push_back(polygon_1);

  // Load textures and cameras poses and intrinsics
  pcl::texture_mapping::CameraVector my_cams;

  const boost::filesystem::path base_dir(argv[3]);
  std::string extension(".txt");
  std::vector<boost::filesystem::path> filenames;
    try {
      for (boost::filesystem::directory_iterator it(base_dir);
           it != boost::filesystem::directory_iterator(); ++it) {
          if (boost::filesystem::is_regular_file(it->status()) &&
              boost::filesystem::extension(it->path()) == extension) {
            filenames.push_back(it->path());
            }
        }
    } catch (const boost::filesystem::filesystem_error& e) {
        cerr << e.what() << endl;
    }
    std::sort(filenames.begin(), filenames.end());


    for(int i=0; i<filenames.size(); ++i){
      std::cout << filenames[i].string() << std::endl;
      pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
      read_cam_pose_file(filenames[i].string(), cam);
      cam.texture_file = filenames[i].stem().string() + ".png";
      my_cams.push_back(cam);
    }

  // Create materials for each texture (and one extra for occluded faces)
  mesh.tex_materials.resize(my_cams.size() + 1);
  for (int i = 0; i <= my_cams.size(); ++i) {
    pcl::TexMaterial mesh_material;
    mesh_material.tex_Ka.r = 0.2f;
    mesh_material.tex_Ka.g = 0.2f;
    mesh_material.tex_Ka.b = 0.2f;

    mesh_material.tex_Kd.r = 0.8f;
    mesh_material.tex_Kd.g = 0.8f;
    mesh_material.tex_Kd.b = 0.8f;

    mesh_material.tex_Ks.r = 1.0f;
    mesh_material.tex_Ks.g = 1.0f;
    mesh_material.tex_Ks.b = 1.0f;

    mesh_material.tex_d = 1.0f;
    mesh_material.tex_Ns = 75.0f;
    mesh_material.tex_illum = 2;

    std::stringstream tex_name;
    tex_name << "material_" << i;
    tex_name >> mesh_material.tex_name;

    if (i < my_cams.size())
      mesh_material.tex_file = my_cams[i].texture_file;
    else
      mesh_material.tex_file = "occluded.jpg";

    mesh.tex_materials[i] = mesh_material;
  }

  // Sort faces
  pcl::TextureMapping<pcl::PointXYZ>
      tm;  // TextureMapping object that will perform the sort
  tm.textureMeshwithMultipleCameras(mesh, my_cams);

  // compute normals for the mesh
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  n.setInputCloud(cloud);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  n.compute(*normals);

  // Concatenate XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

  pcl::toPCLPointCloud2(*cloud_with_normals, mesh.cloud);
  pcl::io::saveOBJFile(argv[2], mesh, 5);

  return 0;
}
