
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/registration_visualizer.h>

#ifdef WIN32
#define sleep(x) Sleep((x)*1000)
#endif

class SViewer{
	pcl::visualization::CloudViewer *viewer;
public:
	SViewer () /*: viewer(new pcl::visualization::CloudViewer("coisas"))*/{ 
		
	}

	// este metodo e invocado sempre que ocorre o evento de uma nova frame
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
		pcl::PointCloud<pcl::PointXYZ> cloud2(*cloud);
       if (!viewer->wasStopped()){
         viewer -> showCloud (cloud);
		 pcl::io::savePCDFileASCII("roda4.pcd", cloud2);
	   }
	}


	// 
	void run(){

		// viewer->registerKeyboardCallback(SViewer::keyEventOcurred, (void*) &viewer);//, (void*) NULL);

		pcl::Grabber* interface = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
		  boost::bind (&SViewer::cloud_cb_, this, _1);

		interface->registerCallback(f);

		interface->start();
		
		// impede que feche logo a aplicacao
		while (!viewer->wasStopped()){
			 sleep (1);
			interface->stop ();
		}
		

       //interface->stop ();
	}

	pcl::PointCloud<pcl::PointXYZ> load(char* file){

		std::cout << "carregar\n" << endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud (new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *loaded_cloud) == -1){
			PCL_ERROR ("Couldn't read file roda \n");
			
		  }else
			  std::cout << "carregou\n" << endl;

		  return *loaded_cloud;
	}

	void show(pcl::PointCloud<pcl::PointXYZ>::ConstPtr Final){
		viewer->showCloud(Final);
	}

	void keyEventOcurred(const pcl::visualization::KeyboardEvent &evento, void* coiso){
		if(evento.keyDown()){
			std::cout << "Key pressed: " << evento.getKeyCode() << "\n"; 
		}
	}

	void cloudRegistration(){
		pcl::PointCloud<pcl::PointXYZ> img1;
		pcl::PointCloud<pcl::PointXYZ> img2;
		pcl::PointCloud<pcl::PointXYZ> img1_filtered;
		pcl::PointCloud<pcl::PointXYZ> img2_filtered;
		pcl::PointCloud<pcl::PointXYZ> Final;
		pcl::RegistrationVisualizer<pcl::PointXYZ, pcl::PointXYZ> registrationVisualizer;
	
		img1 = load("file1.pcd");
		img2 = load("file2s.pcd");

		// Classe que faz o downsampling da imagem
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		// Tamanho da "caixa" que ira servir para compactar os pontos
		sor.setLeafSize (0.02, 0.02, 0.02);

		sor.setInputCloud (img1.makeShared());
		std::cout << "\n inputCloud.size()=" << img1.size() << std::endl;
		sor.filter (img1_filtered);
		std::cout << "\n inputCloudFiltered.size()=" << img1_filtered.size() << std::endl;

		sor.setInputCloud (img2.makeShared());
		std::cout<<"\n targetCloud.size()=" << img2_filtered.size() << std::endl;
		sor.filter (img2_filtered);
		std::cout << "\n targetCloudFiltered.size()=" << img2_filtered.size() << "\n" << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::ConstPtr source = img1_filtered.makeShared();
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr target = img2_filtered.makeShared();

		pcl::PointCloud<pcl::PointXYZ> source_aligned;

		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

		// Define o numero maximo de iteracoes do metodo ICP
		icp.setMaximumIterations(10);
		icp.setMaxCorrespondenceDistance (0.8);
		icp.setRANSACOutlierRejectionThreshold (0.6);

		icp.setInputCloud  (source);
		icp.setInputTarget (target);
		registrationVisualizer.setRegistration (icp);

		// Processo de registo e respectivo resultado
		icp.align (source_aligned);
		std::cout << "has converged:" << icp.hasConverged () << " score: " << icp.getFitnessScore () << std::endl;
		std::cout << icp.getFinalTransformation () << std::endl;
	}

	
};

int main(){
	SViewer sv;
	sv.cloudRegistration();

	getchar();
	return 0;
}
