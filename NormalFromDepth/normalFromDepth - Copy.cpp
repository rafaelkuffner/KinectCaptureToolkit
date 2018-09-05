#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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
#include <pcl/common/common.h>
#include "ICP.h"




using namespace std;
using namespace tinyxml2;

string depthPath = "F:\\Software\\VSProjects\\KinectCPP\\Debug\\Output\\THEREALSWEEP3\\depth";
string outputPath = "F:\\Software\\VSProjects\\KinectCPP\\Debug\\Output\\THEREALSWEEP3\\normals\\outputNormal";
string cameraPath = "F:\\Software\\VSProjects\\KinectCPP\\Debug\\Output\\calibfileKincpp2.xml";
bool kin1 = false;
float width, height;
float distance = 0.01;
float cameraDepthMatrix[3][3];
float cameraColorMatrix[3][3];
float rotateMatrix[3][3];
float translateMatrix[3];

using namespace pcl;



void loadCameraParams(){
	tinyxml2::XMLDocument doc;
	doc.LoadFile(cameraPath.c_str());
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

	const char *r = doc.FirstChildElement()->FirstChildElement("Rotation")->GetText();

	part = strtok((char *)r, " ");
	for (size_t i = 0; i < 3; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			rotateMatrix[i][j] = atof(part);
			part = strtok(NULL, " ");
		}
	}

	const char *t = doc.FirstChildElement()->FirstChildElement("Translation")->GetText();

	part = strtok((char *)t, " ");
	for (size_t j = 0; j < 3; j++)
	{
		translateMatrix[j] = atof(part);
		part = strtok(NULL, " ");
	}
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

void loadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cl, string path){

	//Reading data
	unsigned short depth;

	FILE *f = fopen(path.c_str(), "rb");
	fread(&depth, sizeof (depth), 1, f);

	int i = 0;
	int j = 0;
	if (kin1){
		width = 640;
		height = 480;
	}
	else{
		width = 512;
		height = 424;
	}
	for (i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			pcl::PointXYZ point;

			float z = depth;
			if(z != 0){
				point.x = (float)(z * (j - cameraDepthMatrix[0][2]) / cameraDepthMatrix[0][0]) / 1000;
				point.y = (float)-(z * (i - cameraDepthMatrix[1][2]) / cameraDepthMatrix[1][1]) / 1000;
				point.z = (float) z / 1000;

				cl->points.push_back(point);
			}
			fread(&depth, sizeof (depth), 1, f);
		}
	}
} 

void writeToDisk(pcl::PointCloud<pcl::PointNormal>::Ptr cloud,string path){

	ofstream file;
	file.open(path);
	for(pcl::PointCloud<pcl::PointNormal>::iterator it = cloud->begin(); it != cloud->end(); it++){
		file << it->x << " " << it->y << " " << it->z << " " << it->normal_x << " " << it->normal_y << " " << it->normal_z << "\n";
	}

}

void runProcess(string s){
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	ZeroMemory( &si, sizeof(si) );
	si.cb = sizeof(si);
	ZeroMemory( &pi, sizeof(pi) );

	LPSTR in = const_cast<char *>(s.c_str());
	// Start the child process. 
	if( !CreateProcess( NULL,in, NULL, NULL, FALSE, 0,NULL,NULL,&si,&pi)) 
	{
		printf( "CreateProcess failed (%d).\n", GetLastError() );
		return;
	}

	// Wait until child process exits.
	WaitForSingleObject( pi.hProcess, INFINITE );

	// Close process and thread handles. 
	CloseHandle( pi.hProcess );
	CloseHandle( pi.hThread );
}


void pairAlign (const PointCloud<PointNormal>::Ptr cloud_src, const PointCloud<PointNormal>::Ptr cloud_tgt, PointCloud<PointNormal>::Ptr output, Eigen::Matrix4f &final_transform,float distance, bool downsample = false)
{
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud<PointNormal>::Ptr src (new PointCloud<PointNormal>);
	PointCloud<PointNormal>::Ptr tgt (new PointCloud<PointNormal>);
	pcl::VoxelGrid<PointNormal> grid;
	if (downsample)
	{
		
		grid.setLeafSize (0.020, 0.020, 0.020);
		grid.setInputCloud (cloud_src);
		grid.filter (*src);

		grid.setInputCloud (cloud_tgt);
		grid.filter (*tgt);

	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}

	//
	// Align
	pcl::IterativeClosestPointWithNormals<PointNormal, PointNormal> reg;

	reg.setTransformationEpsilon (1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance (distance);  
	// Set the point representation

	reg.setInputCloud (src);
	reg.setInputTarget (tgt);

	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	PointCloud<PointNormal>::Ptr reg_result = src;
	reg.setMaximumIterations (2);
	float tuneValue = 0.0025;
	for (int i = 0; i <50; ++i)
	{
		src = reg_result;
		reg.setInputCloud (src);
		reg.align (*reg_result);
		Ti = reg.getFinalTransformation () * Ti;
		if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ()){
			reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () -tuneValue);
			if(reg.getMaxCorrespondenceDistance () < tuneValue)
				tuneValue /= 10;
		}
		prev = reg.getLastIncrementalTransformation ();
	}

	// Get the transformation from target to source
	targetToSource = Ti.inverse();
	pcl::transformPointCloudWithNormals (*cloud_tgt, *output, targetToSource);

	//add the source to the transformed target
	//*output += *cloud_src;
	final_transform = targetToSource;
}
int main(int argc, char* argv[]) {
	loadCameraParams();
	vector<string> paths = GetFilesInDirectory(depthPath);
	_Longlong countFile = 0;

	vector<PointCloud<PointNormal>::Ptr> frames;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	
	
	//Load and create clouds and normals
	for (vector<string>::iterator it = paths.begin(); it != paths.end(); it++)
	{
		string dpath = *it;
		loadPointCloud(cloud,dpath);
		PCL_INFO("Calculating normal %s", dpath);
		tree->setInputCloud (cloud);
		n.setInputCloud (cloud);
		n.setSearchMethod (tree);
		n.setKSearch(30);
		//n.setRadiusSearch (0.7);
		n.compute (*normals);
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		frames.push_back(cloud_with_normals);
		cloud->clear();
		normals->clear();

		mesh.cloud = *cloud;
		mesh.header = cloud->header;
	
	}


	float theta = M_PI/4;
	//Align with icp

	pcl::PointCloud<pcl::PointNormal>::Ptr a = frames[0];

	Eigen::Vector4f centroid;

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	pcl::compute3DCentroid(*a,centroid);
	transform(0,3) = -centroid.x();
	transform(2,3) = -centroid.z();

	pcl::PointCloud<pcl::PointNormal>::Ptr result(new PointCloud<PointNormal>);
	pcl::transformPointCloudWithNormals(*a,*a,transform);
	
	*result+= *a;

	pcl::PointCloud<pcl::PointNormal>::Ptr transf (new pcl::PointCloud<pcl::PointNormal> ());
	
		PCL_INFO("Starting align");
	for(int i = 1; i < frames.size() ; i++){
		
		PCL_INFO("Aligning %d\n", i);
		pcl::PointCloud<pcl::PointNormal>::Ptr b = frames[i];

		//rotation cloud
		Eigen::Affine3f transformRot = Eigen::Affine3f::Identity();
		transformRot.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));

		pcl::PointNormal max,min;
		pcl::getMinMax3D<PointNormal>(*b,min,max);
		float sizeX2 = max.x - min.x;
		float sizeZ2 = max.z - min.z;
		Eigen::Matrix4f correct = Eigen::Matrix4f::Identity();
		if(i == 1 || i== 2 || i == 3){
			correct(0,3) -=  sizeZ2/6;
		}
		if(i == 3 || i== 4 || i == 5){
			correct(2,3) +=  sizeZ2/2;
		}
		if(i == 5 || i== 6 || i == 7){
			correct(0,3) +=  sizeZ2/6;
		}

		pcl::transformPointCloudWithNormals(*b,*b,transform);
		pcl::transformPointCloudWithNormals(*b,*b,transformRot);
		pcl::transformPointCloudWithNormals(*b,*b,correct);

		Eigen::Matrix4f transformMat = Eigen::Matrix4f::Identity ();

		pairAlign(result,b,transf,transformMat,0.06,true);
		//frames[i] = transf;
		*b = *transf;
		*result  += *b;
		theta += M_PI/4;
		transf->clear();
	}

	result->clear();
	result = frames[frames.size()-1];
	
		PCL_INFO("Starting reverse align");
	//reverse align
	for(int i = frames.size()-2; i >= 0 ; i--){
		PCL_INFO("Aligning %d\n", i);
		pcl::PointCloud<pcl::PointNormal>::Ptr b = frames[i];
		Eigen::Matrix4f transformMat = Eigen::Matrix4f::Identity ();
		pairAlign(result,b,transf,transformMat,0.025,true);
		//frames[i] = transf;
		*b = *transf;
		*result  += *b;
		transf->clear();
	}
	
	string path = outputPath + "combined";
	writeToDisk(result,path);

	pcl::Poisson<PointNormal> rec;
	rec.setPointWeight(2);
	rec.setSamplesPerNode(2);
	rec.setInputCloud(result);
	pcl::PolygonMesh mesh;
	rec.reconstruct(mesh);
	pcl::io::savePLYFile("total.ply",mesh);
	
	

		//Create surfaces and trim (still need to texture)
		//Faces are swapped, careful, maybe infert normals? 
	for(vector<PointCloud<PointNormal>::Ptr>::iterator it = frames.begin(); it != frames.end(); it++){
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
			"--trim 7 --aRatio 0";
		runProcess(ss2.str());
		countFile++;
	}
}

