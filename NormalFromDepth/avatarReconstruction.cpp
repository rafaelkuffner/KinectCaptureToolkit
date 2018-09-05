#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include "tinyxml2.h"
#include <stdio.h>
#include <string>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/surface/texture_mapping.h>
#include "ICP.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <map>
#include<png.hpp>

using namespace std;
using namespace tinyxml2;
using pcl::visualization::PointCloudColorHandlerCustom;

string emptyDepth = "D:\\Software\\AvatarsProjects\\CaptureServer\\ReconstructionClient\\Release\\Output\\empty\\depth";
string depthPath = "D:\\Software\\AvatarsProjects\\CaptureServer\\ReconstructionClient\\Release\\Output\\1\\depth";
string outputPath = "D:\\Software\\AvatarsProjects\\CaptureServer\\ReconstructionClient\\Release\\Output\\1";
string calibPath = "D:\\Google Drive\\VSProjects\\KinectCPP\\Debug\\Output\\calibfileNovo2.xml";
string colorPath = "D:\\Software\\AvatarsProjects\\CaptureServer\\ReconstructionClient\\Release\\Output\\1\\color ";
string cameraPath = "D:\\Software\\AvatarsProjects\\CaptureServer\\ReconstructionClient\\Release\\Output\\camerafiles";
string calibrationFile = "D:\\Google Drive\\VSProjects\\AvatarsProjects\\CaptureServer\\configSettings.txt";

bool performSeg;
bool kin1 = false;
float width, height;
float distance = 0.01;
float cameraDepthMatrix[3][3];
float cameraColorMatrix[3][3];

map < string, vector<float> > calibration;
Eigen::Matrix4f corrections[4];
using namespace pcl;

std::map<int, string> idCalib;
//utility functions

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}

vector<string> GetFilesInDirectory(const string directory)
{
	HANDLE dir;
	WIN32_FIND_DATA file_data;
	vector<string> out;
	string direc = directory + "/*";
	const char* temp = direc.c_str();

	if ((dir = FindFirstFile(temp, &file_data)) == INVALID_HANDLE_VALUE)
		return out;

	do {
		const string file_name = file_data.cFileName;
		const string full_file_name = directory + "\\" + file_name;
		const bool is_directory = (file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;

		if (file_name[0] == '.')
			continue;

		if (is_directory)
			continue;

		out.push_back(full_file_name);
	} while (FindNextFile(dir, &file_data));

	FindClose(dir);
	return out;
}

void loadConfig(){
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini("config.ini", pt);
	depthPath = pt.get<std::string>("depthPath");
	emptyDepth = pt.get<std::string>("emptyDepth");
	outputPath = pt.get<std::string>("outputPath");
	calibPath = pt.get<std::string>("calibPath");
	colorPath = pt.get<std::string>("colorPath");
	cameraPath = pt.get<std::string>("cameraPath");
	calibrationFile = pt.get<std::string>("calibrationFile");
	int num = pt.get<int>("NUMBER");
	performSeg = pt.get<int>("PERFORM_SEG") == 1? true : false;
	for (int i = 0; i < num; i++){
		stringstream s;
		s << i;
		string calibname = pt.get<string>(s.str());
		idCalib[i] = calibname;
	}
}

void loadCalibParams(){
	tinyxml2::XMLDocument doc;
	doc.LoadFile(calibPath.c_str());
	char *part;
	const char *d = doc.FirstChildElement()->FirstChildElement("Depth_intrinsics")->GetText();

	part = strtok((char *)d, " ");
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{

			cameraDepthMatrix[i][j] = atof(part);
			part = strtok(NULL, " ");
		}
	}

	const char *c = doc.FirstChildElement()->FirstChildElement("Color_intrinsics")->GetText();

	part = strtok((char *)c, " ");
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			cameraColorMatrix[i][j] = atof(part);
			part = strtok(NULL, " ");
		}
	}
}


void loadPointCloudNoFormat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string pointCloudPath){
	std::ifstream f;
	f.open(pointCloudPath, fstream::in);
	std::string line;
	string delimiter1 = ";";
	string delimiter2 = ":";

	string *pos = new string[3];
	string *col = new string[3];
	while (std::getline(f, line))
	{
		replace(line.begin(), line.end(), ',', '.');
		int sep = line.find(delimiter2);
		string s1 = line.substr(0, sep);
		string s2 = line.substr(sep + 1, line.length());
		size_t p;
		int i = 0;
		while ((p = s1.find(delimiter1)) != std::string::npos) {
			pos[i] = s1.substr(0, p);
			s1.erase(0, p + delimiter1.length());
			i++;
		}
		pos[i] = s1;
		i = 0;
		while ((p = s2.find(delimiter1)) != std::string::npos) {
			col[i] = s2.substr(0, p);
			s2.erase(0, p + delimiter1.length());
			i++;
		}
		col[i] = s2;
		pcl::PointXYZ point;
		point.x = stof(pos[0]);
		point.y = stof(pos[1]);
		point.z = stof(pos[2]);
		if (point.x != 0 && point.y != 0 && point.z != 0)
			cloud->points.push_back(point);

	}


}

void loadPointCloudColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cl, string path, string emptybg,string cpath){

	//Reading data
	unsigned short depth;
	unsigned short bgdepth;

	FILE *f = fopen(path.c_str(), "rb");
	FILE *f2 = fopen(emptybg.c_str(), "rb");
	//FILE *f3 = fopen(cpath.c_str(), "rb");
	fread(&depth, sizeof(depth), 1, f);
	fread(&bgdepth, sizeof(depth), 1, f2);
	int i = 0;
	int j = 0;
	int bgThreshold = 150;
	width = 512;
	height = 424;
	png::image< png::rgba_pixel > image(cpath);
	for (i = height - 1; i >= 0; i--){
		for (int j = 0; j < width; j++){
			pcl::PointXYZRGB point;
			float z = depth;
			if (z != 0 && (!performSeg || std::abs(depth - bgdepth)> bgThreshold)){
				point.x = -(float)(z * (j - cameraDepthMatrix[0][2]) / cameraDepthMatrix[0][0]) / 1000;
				point.y = (float)(z * (i - cameraDepthMatrix[1][2]) / cameraDepthMatrix[1][1]) / 1000;
				point.z = (float)z / 1000;
				point.r = image[height-i-1][j].red;
				point.g = image[height-i-1][j].green;
				point.b = image[height-i-1][j].blue;
					
				cl->points.push_back(point);
			}
			fread(&depth, sizeof(depth), 1, f);
			fread(&bgdepth, sizeof(depth), 1, f2);
		}
	}
	/*
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	// build the filter
	outrem.setInputCloud(cl);
	outrem.setRadiusSearch(0.015);
	outrem.setMinNeighborsInRadius(6);
	// apply filter
	outrem.filter(*cl);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cl);
	sor.setMeanK(50);
	sor.setStddevMulThresh(0.03);
	sor.filter(*cl);*/
}

void loadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cl, string path, string emptybg){

	//Reading data
	unsigned short depth;
	unsigned short bgdepth;

	FILE *f = fopen(path.c_str(), "rb");
	FILE *f2 = fopen(emptybg.c_str(), "rb");
	fread(&depth, sizeof(depth), 1, f);
	fread(&bgdepth, sizeof(depth), 1, f2);
	int i = 0;
	int j = 0;
	int bgThreshold = 90;
	width = 512;
	height = 424;

	for (i =height -1; i >= 0; i--){
		for (int j = 0; j < width; j++){
			pcl::PointXYZ point;

			float z = depth;
			if (z != 0 && (!performSeg || std::abs(depth - bgdepth)> bgThreshold)){
				point.x = -(float)(z * (j - cameraDepthMatrix[0][2]) / cameraDepthMatrix[0][0]) / 1000;
				point.y = (float)(z * (i - cameraDepthMatrix[1][2]) / cameraDepthMatrix[1][1]) / 1000;
				point.z = (float)z / 1000;

				cl->points.push_back(point);
			}
			fread(&depth, sizeof(depth), 1, f);
			fread(&bgdepth, sizeof(depth), 1, f2);
		}
	}
	/*
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	// build the filter
	outrem.setInputCloud(cl);
	outrem.setRadiusSearch(0.015);
	outrem.setMinNeighborsInRadius(6);
	// apply filter
	outrem.filter(*cl);
	
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cl);
	sor.setMeanK(50);
	sor.setStddevMulThresh(0.03);
	sor.filter(*cl);*/
}
void writeToDisk(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, string path){

	ofstream file;
	file.open(path);
	for (pcl::PointCloud<pcl::PointNormal>::iterator it = cloud->begin(); it != cloud->end(); it++){
		file << it->x << " " << it->y << " " << it->z << " " << it->normal_x << " " << it->normal_y << " " << it->normal_z << "\n";
	}

}

void writeToDiskWithNormals(pcl::PointCloud<pcl::PointNormal>::Ptr correctedCloud, vector<pcl::Vertices, allocator<pcl::Vertices>> triangles, string path){

	ofstream file;
	file.open(path);
	file << "POINTS X Y Z NX NY NZ\n";
	pcl::PointCloud<pcl::PointNormal>::iterator it2 = correctedCloud->begin();
	while (it2 != correctedCloud->end()){
		file << it2->x << " " << it2->y << " " << it2->z << " " << it2->normal_x << " " << it2->normal_y << " " << it2->normal_z << "\n";
		it2++;
	}
	file << "TRIANGLES\n";
	for (std::vector<pcl::Vertices, allocator<pcl::Vertices>>::iterator it = triangles.begin(); it != triangles.end(); it++){
		file << it->vertices[0] << " " << it->vertices[1] << " " << it->vertices[2] << "\n";
	}
	file.flush();
	file.close();
}

void runProcess(string s){
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	ZeroMemory(&pi, sizeof(pi));

	LPSTR in = const_cast<char *>(s.c_str());
	// Start the child process. 
	if (!CreateProcess(NULL, in, NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi))
	{
		printf("CreateProcess failed (%d).\n", GetLastError());
		return;
	}

	// Wait until child process exits.
	WaitForSingleObject(pi.hProcess, INFINITE);

	// Close process and thread handles. 
	CloseHandle(pi.hProcess);
	CloseHandle(pi.hThread);
}

pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2;
void showCloudsLeft(const PointCloud<PointNormal>::Ptr cloud_target, const PointCloud<PointNormal>::Ptr cloud_source)
{
	p->removePointCloud("vp1_target");
	p->removePointCloud("vp1_source");

	PointCloudColorHandlerCustom<PointNormal> tgt_h(cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointNormal> src_h(cloud_source, 255, 0, 0);
	p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
	p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

	p->spin();
}

void showCloudsLeftColor(const PointCloud<PointXYZRGBNormal>::Ptr cloud_target, const PointCloud<PointXYZRGBNormal>::Ptr cloud_source)
{
	p->removePointCloud("vp1_target");
	p->removePointCloud("vp1_source");

	PointCloudColorHandlerCustom<PointXYZRGBNormal> tgt_h(cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZRGBNormal> src_h(cloud_source, 255, 0, 0);
	p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
	p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

	p->spin();
}

void showCloudsRight(const  PointCloud<PointNormal>::Ptr cloud_target, const  PointCloud<PointNormal>::Ptr cloud_source)
{
	p->removePointCloud("source");
	p->removePointCloud("target");


	PointCloudColorHandlerCustom<PointNormal> tgt_h(cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointNormal> src_h(cloud_source, 255, 0, 0);

	p->addPointCloud(cloud_target, tgt_h, "target", vp_2);
	p->addPointCloud(cloud_source, src_h, "source", vp_2);

	p->spinOnce();
}
void pairAlign(const PointCloud<PointNormal>::Ptr cloud_src, const PointCloud<PointNormal>::Ptr cloud_tgt, PointCloud<PointNormal>::Ptr output, Eigen::Matrix4f &final_transform, float distance, bool downsample = false)
{
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud<PointNormal>::Ptr src(new PointCloud<PointNormal>);
	PointCloud<PointNormal>::Ptr tgt(new PointCloud<PointNormal>);

	pcl::VoxelGrid<PointNormal> grid;
	if (downsample)
	{

		grid.setLeafSize(0.020, 0.020, 0.020);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);

		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);

	}
	else
	{
		*src = *cloud_src;
		*tgt = *cloud_tgt;
	}

	//
	// Align
	pcl::IterativeClosestPointWithNormals<PointNormal, PointNormal> reg;

	reg.setTransformationEpsilon(1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(distance);
	// Set the point representation

	reg.setInputCloud(src);
	reg.setInputTarget(tgt);

	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloud<PointNormal>::Ptr reg_result = src;
	reg.setMaximumIterations(2);
	float tuneValue = 0.0025;
	for (int i = 0; i <150; ++i)
	{
		src = reg_result;
		reg.setInputCloud(src);
		reg.align(*reg_result);
		Ti = reg.getFinalTransformation() * Ti;
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon()){
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - tuneValue);
			if (reg.getMaxCorrespondenceDistance() < tuneValue)
				tuneValue /= 10;
		}
		prev = reg.getLastIncrementalTransformation();

		showCloudsRight(tgt, src);
	}

	// Get the transformation from target to source
	targetToSource = Ti.inverse();
	pcl::transformPointCloudWithNormals(*cloud_tgt, *output, targetToSource);

	//add the source to the transformed target
	//*output += *cloud_src;
	p->spinOnce();
	p->removePointCloud("source");
	p->removePointCloud("target");
	final_transform = targetToSource;
}


void pairAlignFast(const PointCloud<PointNormal>::Ptr cloud_src, const PointCloud<PointNormal>::Ptr cloud_tgt, PointCloud<PointNormal>::Ptr output, Eigen::Matrix4f &final_transform, float distance, bool downsample = false)
{
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud<PointNormal>::Ptr src(new PointCloud<PointNormal>);
	PointCloud<PointNormal>::Ptr tgt(new PointCloud<PointNormal>);

	pcl::VoxelGrid<PointNormal> grid;
	if (downsample)
	{

		grid.setLeafSize(0.020, 0.020, 0.020);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);

		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);

	}
	else
	{
		*src = *cloud_src;
		*tgt = *cloud_tgt;
	}

	//
	// Align
	pcl::IterativeClosestPointWithNormals<PointNormal, PointNormal> reg;

	reg.setTransformationEpsilon(1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(distance);
	// Set the point representation

	reg.setInputCloud(src);
	reg.setInputTarget(tgt);

	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloud<PointNormal>::Ptr reg_result = src;
	reg.setMaximumIterations(30);
	float tuneValue = 0.0025;

	src = reg_result;
	reg.setInputCloud(src);
	reg.align(*reg_result);
	Ti = reg.getFinalTransformation() * Ti;
	prev = reg.getLastIncrementalTransformation();

	// Get the transformation from target to source
	targetToSource = Ti.inverse();
	pcl::transformPointCloudWithNormals(*cloud_tgt, *output, targetToSource);

	//add the source to the transformed target
	//*output += *cloud_src;
	p->spinOnce();
	p->removePointCloud("source");
	p->removePointCloud("target");
	final_transform = targetToSource;
}



//brief Display a 3D representation showing the a cloud and a list of camera with their 6DOf poses 
void showCameras(pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

	// visualization object
	pcl::visualization::PCLVisualizer visu("cameras");

	// add a visual for each camera at the correct pose
	for (int i = 0; i < cams.size(); ++i)
	{
		// read current camera
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam = cams[i];
		double focal = cam.focal_length;
		double height = cam.height;
		double width = cam.width;

		// create a 5-point visual for each camera
		pcl::PointXYZ p1, p2, p3, p4, p5;
		p1.x = 0; p1.y = 0; p1.z = 0;
		double angleX = RAD2DEG(2.0 * atan(width / (2.0*focal)));
		double angleY = RAD2DEG(2.0 * atan(height / (2.0*focal)));
		double dist = 0.75;
		double minX, minY, maxX, maxY;
		maxX = dist*tan(atan(width / (2.0*focal)));
		minX = -maxX;
		maxY = dist*tan(atan(height / (2.0*focal)));
		minY = -maxY;
		p2.x = minX; p2.y = minY; p2.z = dist;
		p3.x = maxX; p3.y = minY; p3.z = dist;
		p4.x = maxX; p4.y = maxY; p4.z = dist;
		p5.x = minX; p5.y = maxY; p5.z = dist;
		p1 = pcl::transformPoint(p1, cam.pose);
		p2 = pcl::transformPoint(p2, cam.pose);
		p3 = pcl::transformPoint(p3, cam.pose);
		p4 = pcl::transformPoint(p4, cam.pose);
		p5 = pcl::transformPoint(p5, cam.pose);
		std::stringstream ss;
		ss << "Cam #" << i + 1;
		visu.addText3D(ss.str(), p1, 0.1, 1.0, 1.0, 1.0, ss.str());

		ss.str("");
		ss << "camera_" << i << "line1";
		visu.addLine(p1, p2, ss.str());
		ss.str("");
		ss << "camera_" << i << "line2";
		visu.addLine(p1, p3, ss.str());
		ss.str("");
		ss << "camera_" << i << "line3";
		visu.addLine(p1, p4, ss.str());
		ss.str("");
		ss << "camera_" << i << "line4";
		visu.addLine(p1, p5, ss.str());
		ss.str("");
		ss << "camera_" << i << "line5";
		visu.addLine(p2, p5, ss.str());
		ss.str("");
		ss << "camera_" << i << "line6";
		visu.addLine(p5, p4, ss.str());
		ss.str("");
		ss << "camera_" << i << "line7";
		visu.addLine(p4, p3, ss.str());
		ss.str("");
		ss << "camera_" << i << "line8";
		visu.addLine(p3, p2, ss.str());
	}

	// add a coordinate system
	visu.addCoordinateSystem(1.0);

	// add the mesh's cloud (colored on Z axis)
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(cloud, "z");
	visu.addPointCloud(cloud, color_handler, "cloud");

	// reset camera
	visu.resetCamera();

	// wait for user input
	visu.spin();
}


void ApplyCalibrationToPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, string calibrationName){

	boost::filesystem::path full_path(boost::filesystem::current_path());

	// Get calibration data
	std::ifstream f;
	f.open( calibrationFile, fstream::in);
	std::string line;
	string delimiter = ";";
	vector<float> translationRotation; // posx, posy, posz, rotx, roty, rotz
	size_t p;
	int i = 0;

	while (std::getline(f, line))
	{
		std::vector<string> tokens = split(line, ';');
		if (tokens[0] == calibrationName){

			translationRotation.push_back(stof(tokens[1]));
			translationRotation.push_back(stof(tokens[2]));
			translationRotation.push_back(stof(tokens[3]));
			translationRotation.push_back(stof(tokens[4]));
			translationRotation.push_back(stof(tokens[5]));
			translationRotation.push_back(stof(tokens[6]));
			translationRotation.push_back(stof(tokens[7]));
			break;
		}
	}

	// compute point cloud position
	Eigen::Quaterniond q = Eigen::Quaterniond(translationRotation[6], translationRotation[3], translationRotation[4], translationRotation[5]);
	Eigen::Vector3d translate(translationRotation[0], translationRotation[1], translationRotation[2]);


	pcl::transformPointCloudWithNormals(*cloud, *cloud, translate, q);

	calibration.insert(std::pair<string, vector<float>>( calibrationName, translationRotation));

	return;
}

void ApplyCalibrationToPointCloudColor(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, string calibrationName){

	boost::filesystem::path full_path(boost::filesystem::current_path());

	// Get calibration data
	std::ifstream f;
	f.open(calibrationFile, fstream::in);
	std::string line;
	string delimiter = ";";
	vector<float> translationRotation; // posx, posy, posz, rotx, roty, rotz
	size_t p;
	int i = 0;

	while (std::getline(f, line))
	{
		std::vector<string> tokens = split(line, ';');
		if (tokens[0] == calibrationName){

			translationRotation.push_back(stof(tokens[1]));
			translationRotation.push_back(stof(tokens[2]));
			translationRotation.push_back(stof(tokens[3]));
			translationRotation.push_back(stof(tokens[4]));
			translationRotation.push_back(stof(tokens[5]));
			translationRotation.push_back(stof(tokens[6]));
			translationRotation.push_back(stof(tokens[7]));
			break;
		}
	}

	// compute point cloud position
	Eigen::Quaterniond q = Eigen::Quaterniond(translationRotation[6], translationRotation[3], translationRotation[4], translationRotation[5]);
	Eigen::Vector3d translate(translationRotation[0], translationRotation[1], translationRotation[2]);


	pcl::transformPointCloudWithNormals(*cloud, *cloud, translate, q);

	calibration.insert(std::pair<string, vector<float>>(calibrationName, translationRotation));

	return;
}

float cameraTransforms[4];
vector<PointCloud<PointXYZRGBNormal>::Ptr> processPathWithColor(vector<string> paths, vector<string> emptybgs,vector<string> cpaths){
	
	vector<PointCloud<PointXYZRGBNormal>::Ptr> frames;


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	//Load and create clouds and normals
	vector<string>::iterator it2 = emptybgs.begin();
	vector<string>::iterator it3 = cpaths.begin();
	int i = 0;
	for (vector<string>::iterator it = paths.begin(); it != paths.end(); it++, it2++, it3++)
	{

		string dpath = *it;
		string epath = *it2;
		string cpath = *it3;
		loadPointCloudColor(cloud, dpath, epath,cpath);
		PCL_INFO("Calculating normal %s\n", dpath);
		tree->setInputCloud(cloud);
		n.setInputCloud(cloud);
		n.setSearchMethod(tree);
		//n.setKSearch(40);
		n.setRadiusSearch (0.7);
		n.compute(*normals);
		n.setViewPoint(0,0,0);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
		ApplyCalibrationToPointCloudColor(cloud_with_normals, idCalib[i]);
		frames.push_back(cloud_with_normals);
		cloud->clear();
		normals->clear();
		i++;
	}
	return frames;
}

vector<PointCloud<PointNormal>::Ptr> processPath(vector<string> paths, vector<string> emptybgs){

	vector<PointCloud<PointNormal>::Ptr> frames;


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	//Load and create clouds and normals
	vector<string>::iterator it2 = emptybgs.begin();
	int i = 0;
	for (vector<string>::iterator it = paths.begin(); it != paths.end(); it++, it2++)
	{

		string dpath = *it;
		string epath = *it2;
		loadPointCloud(cloud, dpath, epath);
		PCL_INFO("Calculating normal %s\n", dpath);
		tree->setInputCloud(cloud);
		n.setInputCloud(cloud);
		n.setSearchMethod(tree);
		n.setKSearch(40);
		//n.setRadiusSearch (0.7);
		n.compute(*normals);
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
		ApplyCalibrationToPointCloud(cloud_with_normals, idCalib[i]);
		frames.push_back(cloud_with_normals);
		cloud->clear();
		normals->clear();
		i++;
	}
	return frames;
}

pcl::texture_mapping::CameraVector loadCameras(float sizeX, float sizeZ){
	pcl::texture_mapping::CameraVector my_cams;
	float theta = 0;
	float thetaZ = 0;
	int i = 1;
	for (int i = 0; i < 4; i++){
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam;

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		float x, y, z;
		switch (i){
		case 0:
			z = -cameraTransforms[0] - 0.05; x = -0.13; y = -0;  theta = 0; thetaZ = 0;
			break;
		case 1:
			x = -cameraTransforms[1] - sizeX / 2 - 0.18; z = sizeZ - 0.05; y = -0.03; theta = M_PI / 2; thetaZ = 0;
			break;
		case 2:
			x = cameraTransforms[2] + sizeX / 2; z = +sizeZ - 0.39; y = 0;; theta = 3 * M_PI / 2; thetaZ = -2 * M_PI / 180;
			break;
		case 3:
			z = cameraTransforms[3] + sizeZ; x = +0.27; y = 0; theta = M_PI; thetaZ = 0;
			break;
		}

		transform.translation() << x, y, z;
		transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
		transform.rotate(Eigen::AngleAxisf(thetaZ, Eigen::Vector3f::UnitZ()));
		Eigen::Matrix4f mat = transform.matrix();

		cam.width = 512;
		cam.height = 424;
		cam.focal_length = -cameraDepthMatrix[0][0];
		cam.pose = mat;
		std::stringstream texName;
		texName << colorPath <<"\\color0," << i << ".png";
		cam.texture_file = texName.str();
		my_cams.push_back(cam);
	}

	return my_cams;

}


pcl::texture_mapping::CameraVector loadCameras(){
	pcl::texture_mapping::CameraVector my_cams;
	float theta = 0;
	float thetaZ = 0;
	int i = 1;
	for (int i = 0; i < idCalib.size(); i++){
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam;


		vector<float>translationRotation = calibration[idCalib[i]];
		Eigen::Quaternionf q = Eigen::Quaternionf(translationRotation[6], translationRotation[3], translationRotation[4], translationRotation[5]);
		Eigen::Vector3f translate(translationRotation[0], translationRotation[1], translationRotation[2]);
		Eigen::Translation<float, 3> transf(translate);
		Eigen::Transform<float, 3, Eigen::Affine> combined = transf * q;
		
	

		Eigen::Matrix4f mat = combined.matrix().cast<float>();
	//	Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
		pcl::texture_mapping::CameraVector temp;
		cam.width = 512;
		cam.height = 424;
		cam.focal_length = -cameraDepthMatrix[0][0];

		cam.pose = mat;
		temp.push_back(cam);

		std::stringstream texName;
		texName << colorPath << "\\color0," << i << ".png";
		cam.texture_file = texName.str();

		my_cams.push_back(cam);

	}

	return my_cams;

}


int main2(int argc, char* argv[]) {
	loadConfig();
	loadCalibParams();
	
	vector<string> paths = GetFilesInDirectory(depthPath);
	vector<string> bgpaths = GetFilesInDirectory(emptyDepth);
	_Longlong countFile = 0;

	p = new pcl::visualization::PCLVisualizer(argc, argv, "Pairwise Incremental Registration");
	p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

	vector<PointCloud<PointNormal>::Ptr> frames = processPath(paths, bgpaths);
	

	//Align with icp
	pcl::PointCloud<pcl::PointNormal>::Ptr a = frames[0];

	pcl::PointCloud<pcl::PointNormal>::Ptr result(new PointCloud<PointNormal>);

	*result += *a;

	pcl::PointCloud<pcl::PointNormal>::Ptr transf(new pcl::PointCloud<pcl::PointNormal>());
	int indexes[5] = { 0, 1, 1, 3, 4 };
	PCL_INFO("Starting align");
	for (int i = 1; i < frames.size(); i++){
		

		PCL_INFO("Aligning %d\n", i);
		pcl::PointCloud<pcl::PointNormal>::Ptr b = frames[indexes[i]];

		showCloudsLeft(b, result);
		Eigen::Matrix4f transformMat = Eigen::Matrix4f::Identity();
		
		//pairAlign(result, b, transf, transformMat, 0.03, false);
		//showCloudsRight(transf, result);
		//corrections[indexes[i]] = transformMat;
		//*b = *transf;
		*result += *b;
		transf->clear();
	}

	/*result->clear();
	*result += *frames[frames.size()-1];

	PCL_INFO("Starting reverse align");
	//reverse align
	for(int i = frames.size()-2; i >= 0 ; i--){
	PCL_INFO("Aligning %d\n", i);
	pcl::PointCloud<pcl::PointNormal>::Ptr b = frames[i];
	Eigen::Matrix4f transformMat = Eigen::Matrix4f::Identity ();
	pairAlign(result,b,transf,transformMat,0.010,true);
	//frames[i] = transf;
	*b = *transf;
	*result  += *b;
	transf->clear();
	}*/
	string path = outputPath + "combined";
	showCloudsLeft(result, result);
	pcl::RadiusOutlierRemoval<pcl::PointNormal> outrem;
	// build the filter
	outrem.setInputCloud(result);
	outrem.setRadiusSearch(0.07);
	outrem.setMinNeighborsInRadius(35);
	// apply filter
	outrem.filter(*result);
	showCloudsLeft(result, result);
	/*
	pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
	sor.setInputCloud (result);
	sor.setMeanK (50);
	sor.setStddevMulThresh (0.28);
	sor.filter (*result);*/

	pcl::PointCloud<PointXYZ>::Ptr res(new PointCloud<PointXYZ>);
	pcl::PolygonMesh mesh2;
	pcl::Poisson<PointNormal> rec;

	rec.setInputCloud(result);
	rec.reconstruct(mesh2);
	rec.setPointWeight(0);
	rec.setSamplesPerNode(13);
	rec.setDepth(10);
	pcl::fromPCLPointCloud2(mesh2.cloud, *res);


	//TEXTURING
	pcl::TextureMesh mesh;
	mesh.cloud = mesh2.cloud;
	mesh.tex_polygons.push_back(mesh2.polygons);

	pcl::PointXYZ maxr, minr;
	pcl::getMinMax3D<PointXYZ>(*res, minr, maxr);
	//pcl::texture_mapping::CameraVector my_cams = loadCameras(maxr.x-minr.x,maxr.z-minr.z);
	pcl::texture_mapping::CameraVector my_cams = loadCameras();
	mesh.tex_materials.resize(my_cams.size() + 1);
	for (int i = 0; i <= my_cams.size(); ++i)
	{
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
			mesh_material.tex_file = "occluded.png";

		mesh.tex_materials[i] = mesh_material;
	}
	pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
	showCameras(my_cams, res);
	tm.textureMeshwithMultipleCameras(mesh, my_cams);

	pcl::PointCloud<pcl::Normal>::Ptr normalsFinal(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> nn;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr treen(new pcl::search::KdTree<pcl::PointXYZ>);
	treen->setInputCloud(res);
	nn.setInputCloud(res);
	nn.setSearchMethod(treen);
	nn.setKSearch(20);
	nn.compute(*normalsFinal);
	// Concatenate XYZ and normal fields
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*res, *normalsFinal, *cloud_with_normals);
	pcl::toPCLPointCloud2(*cloud_with_normals, mesh.cloud);
	pcl::io::saveOBJFile("test.obj", mesh);
	//pcl::io::savePLYFile("total.ply",mesh2);
	//writeToDiskWithNormals(res,triangs,path);

	//Create surfaces and trim (still need to texture)
	/*for(vector<PointCloud<PointNormal>::Ptr>::iterator it = frames.begin(); it != frames.end(); it++){
	pcl::PointCloud<pcl::PointNormal>::Ptr f = *it;

	string path = outputPath + std::to_string(countFile);
	writeToDisk(f,path);
	stringstream ss;

	ss <<"PoissonRecon.x64.exe --in " << outputPath<< countFile
	<<" --out " << outputPath << "surf" << countFile << ".ply "<<
	"--depth 10 --samplesPerNode 2 --pointWeight 2 --density";
	runProcess(ss.str());
	stringstream ss2;
	ss2 <<"SurfaceTrimmer.x64.exe --in " << outputPath << "surf" << countFile << ".ply"
	<<" --out " << outputPath << "trimmed" << countFile << ".ply "<<
	"--trim 6.2 --aRatio 0";
	runProcess(ss2.str());
	countFile++;
	}*/
}


int main(int argc, char*argv[]){
	loadConfig();
	loadCalibParams();

	vector<string> paths = GetFilesInDirectory(depthPath);
	vector<string> bgpaths = GetFilesInDirectory(emptyDepth);
	vector<string> cpaths = GetFilesInDirectory(colorPath);
	_Longlong countFile = 0;

	p = new pcl::visualization::PCLVisualizer(argc, argv, "Pairwise Incremental Registration");
	p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

	vector<PointCloud<PointXYZRGBNormal>::Ptr> frames = processPathWithColor(paths, bgpaths,cpaths);

	//Align with icp
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr result(new PointCloud<PointXYZRGBNormal>);
	PCL_INFO("Starting align");
	for (int i = 0; i < frames.size(); i++){
		*result += *frames[i];
	}

	string path = outputPath + "combined";
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> outrem;
	// build the filter
	outrem.setInputCloud(result);
	outrem.setRadiusSearch(0.07);
	outrem.setMinNeighborsInRadius(35);
	// apply filter
	outrem.filter(*result);
	showCloudsLeftColor(result, result);
	
	
	pcl::io::savePLYFile(outputPath + "\\total.ply",*result);

	stringstream ss;

		ss << "PoissonRecon.x64.exe --in " << outputPath << "\\total.ply"
		<< " --out " << outputPath << "\\recons" << ".ply " <<
		"--depth 10 --samplesPerNode 2 --pointWeight 2 --density --color 16";
	runProcess(ss.str());
	stringstream ss2;
	ss2 << "SurfaceTrimmer.x64.exe --in " << outputPath << "\\recons"  << ".ply"
		<< " --out " << outputPath << "\\trimmed" << ".ply " <<
		"--trim 6.2 --aRatio 0";
	runProcess(ss2.str());
	getchar();
	return 0;
}
