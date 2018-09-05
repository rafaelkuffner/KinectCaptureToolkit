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



using namespace std;
using namespace tinyxml2;

string depthPath = "F:\\Software\\VSProjects\\KinectCPP\\Debug\\Output\\THEREALSWEEP3\\depth";
string outputPath = "F:\\Software\\VSProjects\\KinectCPP\\Debug\\Output\\THEREALSWEEP3\\normals\\outputNormal";
string cameraPath = "F:\\Software\\VSProjects\\KinectCPP\\Debug\\Output\\calibfileKincpp2.xml";
string colorPath = "F:\\Software\\VSProjects\\KinectCPP\\Debug\\Output\\THEREALSWEEP3\\color\\color";
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

void loadPointCloudNoFormat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,string pointCloudPath){
	std::ifstream f;
	f.open(pointCloudPath,fstream::in);
	std::string line;
	string delimiter1 = ";";
	string delimiter2 = ":";

	string *pos = new string[3];
	string *col = new string[3];
	while (std::getline(f, line))
	{
		replace(line.begin(),line.end(),',','.');
		int sep = line.find(delimiter2);
		string s1 = line.substr(0,sep);
		string s2 = line.substr(sep+1,line.length());
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
		if(point.x != 0 && point.y != 0 && point.z != 0 )
			cloud->points.push_back(point);

	}
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


void writeToDiskWithNormals(pcl::PointCloud<pcl::PointNormal>::Ptr correctedCloud, vector<pcl::Vertices,allocator<pcl::Vertices>> triangles, string path){

	ofstream file;
	file.open(path);
	file<<"POINTS X Y Z NX NY NZ\n";
	pcl::PointCloud<pcl::PointNormal>::iterator it2 = correctedCloud->begin();
	while(it2 != correctedCloud->end()){
		file << it2->x << " " << it2->y << " " << it2->z << " " << it2->normal_x << " " <<  it2->normal_y << " " <<  it2->normal_z <<"\n";
		it2++;
	}
	file<<"TRIANGLES\n";
	for(std::vector<pcl::Vertices,allocator<pcl::Vertices>>::iterator it = triangles.begin(); it != triangles.end();it++){
		file << it->vertices[0] << " " <<it->vertices[1]<< " " << it->vertices[2] << "\n";
	}
	file.flush();
	file.close();
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
	for (int i = 0; i <150; ++i)
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




//brief Display a 3D representation showing the a cloud and a list of camera with their 6DOf poses 
void showCameras (pcl::texture_mapping::CameraVector cams, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{

	// visualization object
	pcl::visualization::PCLVisualizer visu ("cameras");

	// add a visual for each camera at the correct pose
	for(int i = 0 ; i < cams.size () ; ++i)
	{
		// read current camera
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam = cams[i];
		double focal = cam.focal_length;
		double height = cam.height;
		double width = cam.width;

		// create a 5-point visual for each camera
		pcl::PointXYZ p1, p2, p3, p4, p5;
		p1.x=0; p1.y=0; p1.z=0;
		double angleX = RAD2DEG (2.0 * atan (width / (2.0*focal)));
		double angleY = RAD2DEG (2.0 * atan (height / (2.0*focal)));
		double dist = 0.75;
		double minX, minY, maxX, maxY;
		maxX = dist*tan (atan (width / (2.0*focal)));
		minX = -maxX;
		maxY = dist*tan (atan (height / (2.0*focal)));
		minY = -maxY;
		p2.x=minX; p2.y=minY; p2.z=dist;
		p3.x=maxX; p3.y=minY; p3.z=dist;
		p4.x=maxX; p4.y=maxY; p4.z=dist;
		p5.x=minX; p5.y=maxY; p5.z=dist;
		p1=pcl::transformPoint (p1, cam.pose);
		p2=pcl::transformPoint (p2, cam.pose);
		p3=pcl::transformPoint (p3, cam.pose);
		p4=pcl::transformPoint (p4, cam.pose);
		p5=pcl::transformPoint (p5, cam.pose);
		std::stringstream ss;
		ss << "Cam #" << i+1;
		visu.addText3D(ss.str (), p1, 0.1, 1.0, 1.0, 1.0, ss.str ());

		ss.str ("");
		ss << "camera_" << i << "line1";
		visu.addLine (p1, p2,ss.str ());
		ss.str ("");
		ss << "camera_" << i << "line2";
		visu.addLine (p1, p3,ss.str ());
		ss.str ("");
		ss << "camera_" << i << "line3";
		visu.addLine (p1, p4,ss.str ());
		ss.str ("");
		ss << "camera_" << i << "line4";
		visu.addLine (p1, p5,ss.str ());
		ss.str ("");
		ss << "camera_" << i << "line5";
		visu.addLine (p2, p5,ss.str ());
		ss.str ("");
		ss << "camera_" << i << "line6";
		visu.addLine (p5, p4,ss.str ());
		ss.str ("");
		ss << "camera_" << i << "line7";
		visu.addLine (p4, p3,ss.str ());
		ss.str ("");
		ss << "camera_" << i << "line8";
		visu.addLine (p3, p2,ss.str ());
	}

	// add a coordinate system
	visu.addCoordinateSystem (1.0);

	// add the mesh's cloud (colored on Z axis)
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler (cloud, "z");
	visu.addPointCloud (cloud, color_handler, "cloud");

	// reset camera
	visu.resetCamera ();

	// wait for user input
	visu.spin ();
}

float cameraTransforms[4];
Eigen::Matrix4f alignTransforms[4];

vector<PointCloud<PointNormal>::Ptr> processPath(vector<string> paths){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	vector<PointCloud<PointNormal>::Ptr> frames;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);


	//Load and create clouds and normals
	for (vector<string>::iterator it = paths.begin(); it != paths.end(); it++)
	{
		string dpath = *it;
		loadPointCloud(cloud,dpath);
		PCL_INFO("Calculating normal %s\n", dpath);
		tree->setInputCloud (cloud);
		n.setInputCloud (cloud);
		n.setSearchMethod (tree);
		n.setKSearch(40);
		//n.setRadiusSearch (0.7);
		n.compute (*normals);
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		frames.push_back(cloud_with_normals);
		cloud->clear();
		normals->clear();
		
	}
	return frames;
}

pcl::texture_mapping::CameraVector loadCameras(){
	pcl::texture_mapping::CameraVector my_cams;
	float theta =0;
	float thetaZ=0;
	int i =2;
	for(int i = 0; i < 4 ; i++){
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
		
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		float x,y,z;
		switch(i){
		case 0:
			z = -cameraTransforms[0]; x = 0; theta = 0;
			break;
		case 1:
			x = -cameraTransforms[1]; z = 0; theta = M_PI/2;
			break;
		case 2:
			 x = cameraTransforms[2]; z = 0; theta = 3*M_PI/2;
			break;
		case 3:
			z = cameraTransforms[3];x = 0; theta = M_PI;
			break;
		}

		transform.translation() << x, 0, z;
		transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));

		Eigen::Matrix4f mat = transform.matrix();
		alignTransforms[i](0,1) = 0;
		alignTransforms[i](0,2) = 0;
		alignTransforms[i](0,3) = 0;
		mat *= alignTransforms[i];

		cam.width = 512;
		cam.height = 424;
		cam.focal_length = -cameraDepthMatrix[0][0];
		cam.pose = mat;
		std::stringstream texName;
		texName << colorPath << i << ".png";
		cam.texture_file = texName.str();
		my_cams.push_back(cam);
	}
	
	return my_cams;
	
}


int main(int argc, char* argv[]) {
	loadCameraParams();
	vector<string> paths = GetFilesInDirectory(depthPath);
	_Longlong countFile = 0;

	vector<PointCloud<PointNormal>::Ptr> frames = processPath(paths);


	float theta = M_PI/2;
	//Align with icp
	pcl::PointCloud<pcl::PointNormal>::Ptr a = frames[0];

	Eigen::Vector4f centroid;
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	pcl::PointNormal maxx,minn;
	pcl::getMinMax3D<PointNormal>(*a,minn,maxx);
	float sizeZ = maxx.z - minn.z;
	transform(2,3) = -minn.z;
	cameraTransforms[0] = minn.z;
	alignTransforms[0] = Eigen::Matrix4f::Identity();
	pcl::PointCloud<pcl::PointNormal>::Ptr result(new PointCloud<PointNormal>);
	pcl::transformPointCloudWithNormals(*a,*a,transform);


	*result+= *a;

	pcl::PointCloud<pcl::PointNormal>::Ptr transf (new pcl::PointCloud<pcl::PointNormal> ());

	PCL_INFO("Starting align");
	for(int i = 1; i < frames.size() ; i++){

		PCL_INFO("Aligning %d\n", i);
		pcl::PointCloud<pcl::PointNormal>::Ptr b = frames[i];

		pcl::PointNormal max,min;
		pcl::getMinMax3D<PointNormal>(*b,min,max);
		float sizeX2 = max.x - min.x;
		float sizeZ2 = max.z - min.z;
		Eigen::Matrix4f correct = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f transformCenter = Eigen::Matrix4f::Identity();
		transformCenter(2,3) = -min.z;
		if(i== 1){
			correct(0,3) -=  sizeZ2/4;
			correct(2,3) +=  sizeZ2/4;
			theta = M_PI/2;
		}
		if( i == 3){
			correct(2,3) +=  sizeZ2;
			theta = M_PI;
		}
		if( i == 2){
			correct(0,3) +=  sizeZ2/4;
			correct(2,3) +=  sizeZ2/4;
			theta = -M_PI/2;
		}

		//rotation cloud
		Eigen::Affine3f transformRot = Eigen::Affine3f::Identity();
		transformRot.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
		
		pcl::transformPointCloudWithNormals(*b,*b,transformCenter);
		pcl::transformPointCloudWithNormals(*b,*b,transformRot);
		pcl::transformPointCloudWithNormals(*b,*b,correct);

		Eigen::Matrix4f transformMat = Eigen::Matrix4f::Identity ();

		pairAlign(result,b,transf,transformMat,0.05,true);
		//transformMat *= correct;
		cameraTransforms[i] = min.z;
		alignTransforms[i] =transformMat; 
		//frames[i] = transf;
		*b = *transf;
		*result  += *b;
		theta += M_PI/2;
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

	pcl::PointCloud<PointXYZ>::Ptr res (new PointCloud<PointXYZ>);
	pcl::PolygonMesh mesh2;
	pcl::Poisson<PointNormal> rec;
	rec.setPointWeight(2);
	rec.setSamplesPerNode(2);
	rec.setInputCloud(result);
	rec.reconstruct(mesh2);

	pcl::fromPCLPointCloud2(mesh2.cloud,*res);

	//TEXTURING
	pcl::TextureMesh mesh;
	mesh.cloud = mesh2.cloud;
	mesh.tex_polygons.push_back(mesh2.polygons);

	pcl::texture_mapping::CameraVector my_cams = loadCameras();

	  mesh.tex_materials.resize (my_cams.size () + 1);
	 for(int i = 0 ; i <= my_cams.size() ; ++i)
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

		if(i < my_cams.size ())
		  mesh_material.tex_file = my_cams[i].texture_file;
		else
		  mesh_material.tex_file = "occluded.jpg";

		mesh.tex_materials[i] = mesh_material;
	  }
	pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
	showCameras(my_cams,res);
	tm.textureMeshwithMultipleCameras(mesh, my_cams);
	
	pcl::PointCloud<pcl::Normal>::Ptr normalsFinal (new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> nn;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr treen (new pcl::search::KdTree<pcl::PointXYZ>);
	treen->setInputCloud (res);
	nn.setInputCloud (res);
	nn.setSearchMethod (treen);
	nn.setKSearch (20);
	nn.compute (*normalsFinal);
	// Concatenate XYZ and normal fields
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*res, *normalsFinal, *cloud_with_normals);
	pcl::toPCLPointCloud2 (*cloud_with_normals, mesh.cloud);
	pcl::io::saveOBJFile("test.obj",mesh);
	pcl::io::savePLYFile("total.ply",mesh2);
	//writeToDiskWithNormals(res,triangs,path);

	//Create surfaces and trim (still need to texture)
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
			"--trim 6.2 --aRatio 0";
		runProcess(ss2.str());
		countFile++;
	}
}


