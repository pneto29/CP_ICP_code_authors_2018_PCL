#include <iostream>
using namespace std;


#include <pcl/io/ply_io.h>

#include <algorithm>
#include <string> 
#include <vector>
using std::vector;

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <boost/thread/thread.hpp>

#include <pcl/console/time.h> 
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

bool comparePoint(pcl::PointXYZ p1, pcl::PointXYZ p2);
bool comparePoint(pcl::PointXYZ p1, pcl::PointXYZ p2) { 
        if (p1.z < p2.z) { 
		//if (p1.x < p2.x) { 
		//if (p1.y < p2.y) { 
                return true; 
        } 
        else { 
                return false; 
        } 
} 

vector<int> linspace(int a, int b, int n) {
    vector<int> array;
    double step = (b-a) / (n-1);

    while(a <= b) {
        array.push_back(a);
        a += step;           // could recode to better handle rounding errors
    }
    return array;
}

Eigen::Vector3f getMedianPoint(PointCloud::Ptr cloud){
	Eigen::Vector4f min;
	Eigen::Vector4f max;
	getMinMax3D(*cloud,min,max);

	Eigen::Vector3f med;
	med.x() = (min.x()+max.x())/2;
	med.y() = (min.y()+max.y())/2;
	med.z() = (min.z()+max.z())/2;


	//cout << "Min value: " << min << endl;
	//cout << "Max value: " << max << endl;
    //cout << "Med value: " << med << endl << endl << endl;

	return med;
}

vector<double> min(vector<double> vec){
	double m = vec[0];
	int n = vec.size();
	double k = 0;
	for(int i=0; i<n; i++){
        if(vec[i]<m){
			m = vec[i];
			k = i;
		}
    }

	vector <double> vf;
	vf.push_back(k);
	vf.push_back(m);
	return vf;
}

std::vector<PointCloud::Ptr> cloudPartitioning(PointCloud::Ptr cloud, int n){	

	float s = cloud->width;
	float num = floor(s/n);

	//std::cout << "Size cloud: " << s << std::endl;
	//std::cout << "Num of subclouds: " << n << std::endl;
	//std::cout << "Num of points per subclouds: " << num << std::endl;

	//Sorting
	std::sort(cloud->points.begin(),cloud->points.end(),comparePoint);

	//Creating subclouds
	vector<PointCloud::Ptr> sn;
	for(int j=0;j<n;j++){
		PointCloud::Ptr sn1 (new PointCloud);
		sn1->width = num;
		sn1->height = 1;
		//sn1->resize(sn1->width*sn1->height);

		for(int i=j*num ; i< (j+1)*num ; i++){
			sn1->points.push_back(cloud->points[i]);
		}
		sn.push_back(sn1);
		//std::cout << "Size of sn[" << j << "]: " << sn1->size() << std::endl;
	}
	//std::cout << "Size of sn: " << sn.size() << endl;
	return sn;
}


double computeCloudRMS(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, double max_range){ 
//double computeCloudRMS(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, pcl::PointCloud<pcl::PointXYZ>::ConstPtr source){ 

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>); 
        tree->setInputCloud(target); 

        double fitness_score = 0.0; 

        std::vector<int> nn_indices (1); 
        std::vector<float> nn_dists (1); 

        // For each point in the source dataset 
        int nr = 0; 
        for (size_t i = 0; i < source->points.size (); ++i){ 
                //Avoid NaN points as they crash nn searches 
                if(!pcl_isfinite((*source)[i].x)){ 
                        continue; 
                } 

                // Find its nearest neighbor in the target 
                tree->nearestKSearch (source->points[i], 1, nn_indices, nn_dists); 

                // Deal with occlusions (incomplete targets) 
                if (nn_dists[0] <= max_range*max_range){ 
                        // Add to the fitness score 
                        fitness_score += nn_dists[0]; 						
                        nr++; 
                } 
        } 

        if (nr > 0){ 
			//cout << "nr: " << nr << endl;
			//cout << "fitness_score: " << fitness_score << endl;			
                return sqrt(fitness_score / nr); 
        }else{ 
                return (std::numeric_limits<double>::max ()); 
        } 
}




int
 main (int argc, char** argv)
{
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud_out (new PointCloud);
  PointCloud::Ptr cloud_icp (new PointCloud);

   pcl::console::TicToc time;

  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("dragon0_cor.pcd", *cloud_out) == -1) //* load the file
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("buddha0_cor.pcd", *cloud_out) == -1) //* load the file
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("buddha0.pcd", *cloud_out) == -1) //* load the file
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("bunny0.pcd", *cloud_out) == -1) //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("horse0.pcd", *cloud_out) == -1) //* load the file
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("hammer0.pcd", *cloud_out) == -1) //* load the file
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("jug0.pcd", *cloud_out) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file model \n");
    return (-1);
  }
  
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("dragon24_cor.pcd", *cloud_in) == -1) //* load the file
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("buddha24_cor.pcd", *cloud_in) == -1) //* load the file
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("buddha24.pcd", *cloud_in) == -1) //* load the file
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("bunny45.pcd", *cloud_in) == -1) //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("horse180.pcd", *cloud_in) == -1) //* load the file
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("hammer45.pcd", *cloud_in) == -1) //* load the file
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("jug180.pcd", *cloud_in) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file shape \n");
    return (-1);
  }  

  //-----------------------------------

  
  time.tic();
  //Localizar centróides
  Eigen::Vector3f med_in;
  Eigen::Vector3f med_out;

  med_in = getMedianPoint(cloud_in);
  med_out = getMedianPoint(cloud_out);

  PointCloud::Ptr pm_in (new PointCloud);
  pcl::PointXYZ pc_in;

  pc_in.x = med_in.x();
  pc_in.y = med_in.y();
  pc_in.z = med_in.z();  
  pm_in->points.push_back(pc_in);

  PointCloud::Ptr pm_out (new PointCloud);
  pcl::PointXYZ pc_out;
  pc_out.x = med_out.x();
  pc_out.y = med_out.y();
  pc_out.z = med_out.z();  
  pm_out->points.push_back(pc_out);
    

  // Inicial rotation
  Eigen::Affine3f transform_in = Eigen::Affine3f::Identity();
  Eigen::Affine3f transform_out = Eigen::Affine3f::Identity();

  Eigen::Affine3f transform_in2 = Eigen::Affine3f::Identity();
  Eigen::Affine3f transform_out2 = Eigen::Affine3f::Identity();

  //-----------------------------------

  // Define a translation of 2.5 meters on the x axis.  
  transform_in.translation()  << -med_in.x(), -med_in.y(), -med_in.z();
  transform_out.translation() << -med_out.x(), -med_out.y(), -med_out.z();

  double t_cen = time.toc();
  cout << "Tempo transformacao centroide: " << t_cen << endl << endl;

  // Executing the transformation  
  pcl::transformPointCloud (*cloud_in, *cloud_in, transform_in);
  pcl::transformPointCloud (*cloud_out, *cloud_out, transform_out);


  // The same rotation matrix as before; theta radians arround Z axis
  float theta = -M_PI; // The angle of rotation in radians
  //float theta = -M_PI/4; // The angle of rotation in radians
  transform_in2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  
  pcl::transformPointCloud (*cloud_in, *cloud_in, transform_in2);  

  // Applying cloud partitioning
  int num = 200;
  vector <PointCloud::Ptr> sn_in;
  vector <PointCloud::Ptr> sn_out;
  
  vector <double> vec_t;

  PointCloud tempo;
  tempo.width    = num;
  tempo.height   = 1;  
  tempo.points.resize (tempo.width * tempo.height);

  vector <double> vec_rms_ind;
  vector <double> vec_rms_val;

  for(int n=2; n<num+1; n++){
  //int n = 100;
   
  sn_in = cloudPartitioning(cloud_in,n);
  sn_out = cloudPartitioning(cloud_out,n);
 

  vector <PointCloud::Ptr> sn_final;
  vector <double> rms;  
  
  Eigen::Matrix4f rotation_matrix;

  // ICP
  //time.tic();
  for(int i=1; i<n; i++){
  //for(int i=n-1; i>0; i=i-1){
	  time.tic();

	  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setInputCloud(sn_in[i]);
      icp.setInputTarget(sn_out[i]);
      icp.setMaximumIterations(30);

      icp.align(*cloud_icp);      

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);	  
	  	  
	  Eigen::Matrix4f transform_icp = icp.getFinalTransformation();      	  
	  pcl::transformPointCloud (*cloud_in, *cloud_final, transform_icp);
	  sn_final.push_back(cloud_final);	  
	  double this_rms = computeCloudRMS(cloud_out,cloud_final,std::numeric_limits<double>::max ());
	  rms.push_back(this_rms);
	  //cout<< "RMS " << i << ": " << this_rms << endl << endl;	  
	  
	  if(this_rms<0.0105){ //Horse
	  //if(this_rms<0.007){ //Hammer
	  //if(this_rms<0.002){ //Dragon	  
	  //if(this_rms<0.0029){ //Buddha
		  rotation_matrix = icp.getFinalTransformation();
		  break;
	  }
	double t = time.toc();  
	//cout << "Time icp[" << i << "]:" << t << endl;	
	
	tempo.points[n-1].x = t;
	vec_t.push_back(t);
  } 
  
  cout << "Iteration n = " << n << endl << endl;

  //double t = time.toc();  
  //tempo.points[n-1].x = t;
  pcl::io::savePCDFileASCII("tempoFull_horse2.pcd",tempo);

  //cout << "Time icp:" << t << endl;	
  
  //time.tic();
  vector <double> rms_min = min(rms);
  //double t_rmsmin = time.toc();

  //cout << endl << "Time min: " << t_rmsmin << endl << endl;

  cout << "It " << n << ":" << endl;
  cout << "Idx: " << rms_min[0] << "/ Value: " << rms_min[1] << endl;
  cout << "Tempo ICP: " << t << endl;
  cout << rotation_matrix << endl;
  cout << "-------------------" << endl << endl;

  
  
  //vec_rms_ind.push_back(rms_min[0]);
  //vec_rms_val.push_back(rms_min[1]);
  
  }  // Laço n

  //vector<double> t_min = min(vec_t);
  //vector<double> rms_min_ind = min(vec_rms_ind);
  //vector<double> rms_min_val = min(vec_rms_val);

  //cout << "Menor tempo: " << endl;
  //cout << "N: " << t_min[0]+1 << " / Val: " << t_min[1] << endl << endl;;

  //cout << "Menor rms: " << endl;
  //cout << "Idx: " << rms_min_val[0] << " / Val: " << rms_min_val[1] << endl << endl;;

  ///*

  //---------------------------
  //Visualização
  //---------------------------

  int sizePoints = 2;

  //Primeiro view
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Pre ICP"));
  viewer->setBackgroundColor(255,255,255);
  viewer->addPointCloud(cloud_in,"cloud_in");
  viewer->addPointCloud(cloud_out,"cloud_out");

  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"cloud_out");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_out");  
  
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"cloud_in");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_in");  

  //Segundo view - As nuvens depois do ICP
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("Pos ICP"));  
  
  // Adicionando ambas as nuvens de pontos
  viewer2->setBackgroundColor(255,255,255);

  viewer2->addPointCloud(cloud_out, "cloud_out");
  
  //Configurando cloud_out
  viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"cloud_out");
  viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_out");  

  //viewer2->addPointCloud(cloud_in,"cloud_in");
  //viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0,"cloud_in");
  //viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sizePoints, "cloud_in");  
  

  //Add median points
  //viewer2->addPointCloud(pm_in,"pm_in");
  //viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "pm_in");
  //viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,255,255,"pm_in");

  //viewer2->addPointCloud(pm_out,"pm_out");
  //viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "pm_out");
  //viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255,"pm_out");

  //*/

  //Add subclouds
  
  /*
  for(int i=0;i<n;i++){

	  //cout << "Sn[" << i << "]: " << "sn"+ to_string((_Longlong)i) << endl;

	  float ran   = (0.2 + (0.8 * rand () / (RAND_MAX + 1.0f)));
	  float ran2  = (0.4 + (0.6 * rand () / (RAND_MAX + 1.0f)));
	  float ran3  = (0.6 + (0.4 * rand () / (RAND_MAX + 1.0f)));
	  //cout << "Random color: " << ran << endl;

	  //viewer2->addPointCloud(sn_final[i], "sn"+ to_string((_Longlong)i));
	  viewer2->addPointCloud(sn_in[i], "sn"+ to_string((_Longlong)i));
	  viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,ran3,ran,ran2,"sn"+ to_string((_Longlong)i));	  
	  //viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,255,0,"sn"+ to_string((_Longlong)i));
	  viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sn"+ to_string((_Longlong)i));
  } 
  */

  /*
  
  int i = rms_min[0];
  //for(int k=0; k<cloud_out->points.size()-1; k++){
	  //sn_final[i]->points.push_back(cloud_out->points[k]);
	  //cloud_out->points[k];
  //}
    
  viewer2->addPointCloud(sn_final[i], "sn_final"+ to_string((_Longlong)i));
  viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,255,0,"sn_final"+ to_string((_Longlong)i));	  
  viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sn_final"+ to_string((_Longlong)i));
  
  //Nuvem final  
  //*sn_final[i] += *cloud_out;
  //pcl::io::savePCDFileASCII("CPICP_DRAGON.pcd", *sn_final[i]);
  //pcl::io::savePCDFileASCII("CPICP_BUDDHA.pcd", *sn_final[i]);
  //pcl::io::savePCDFileASCII("CPICP_HORSE.pcd", *sn_final[i]);
  //pcl::io::savePCDFileASCII("CPICP_HAMMER.pcd", *sn_final[i]);

  //std::vector<pcl::visualization::Camera> cam; 

  //Get the camera
  //viewer2->getCameras(cam); 

  //Change camera's position
  //viewer2->setCameraPosition(0,0,1,0,1,0,0);
  */
  while(!viewer->wasStopped())
  {
      viewer->spinOnce();
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  }

  /*
  while(!viewer2->wasStopped())
  {
      viewer2->spinOnce();
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  }
    
  */
  system("pause");
}