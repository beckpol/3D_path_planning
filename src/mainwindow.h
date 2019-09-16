#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <math.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <vtkRenderWindow.h>

#define NMAX 100

#include <QMainWindow>
#include <QGraphicsScene>
#include <QtCore>
#include <QtGui>
#include <QStatusBar>
#include <QMutex>
#include <ui_mainwindow.h>

#define PI 3.14
#define N 30
#define M 30
#define K 10  

typedef pcl::PointXYZ PointT;

struct PlaneT
{
  pcl::PointCloud<PointT>::Ptr points; 
  pcl::ModelCoefficients::Ptr coefficients; 
};

struct ZoneT{
  ZoneT *next;
  int datap;
  int datan;
  int med;
  int id_z;
  int dim_z;

  ZoneT(int p, int n, int idz , int dim , ZoneT* nxt){
    datap=p;
    datan=n;
    med = (n-p)/2+p;
    next = nxt;
    id_z=idz;
    dim_z=dim;
  }
};


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    
    ~MainWindow ()
    {
      if(kinect->isRunning())
        kinect->stop();
      delete ui;
    };
    
    void callback_cloud (const pcl::PointCloud<PointT>::ConstPtr& cloud);
    void processing();
    bool verbose;
    
protected:
    Ui::MainWindow *ui;
    QGraphicsScene *arrowScene;
    QGraphicsPolygonItem *p_arrow;
    QGraphicsScene *radarScene;
    QGraphicsRectItem *p_rec;
    qreal cur_angle;
    qreal angle;
    int v_pt2fw;
    int v_encum;
    float v_offset;
    bool nopoints;
    bool data_modified;
    bool capture;
    ZoneT *zones[K];
    ZoneT *path;
    ZoneT* Insert_zone(ZoneT *zone, int prev, int next, int id, int dim);
    //void Get_MapNoise(float p);
    //void Get_MapObstacle(int minx, int maxx, int minz ,int maxz);

    //PCL code
    
    //variabili iniziali
    pcl::Grabber* kinect;
    pcl::PointCloud<PointT>::ConstPtr get_cloud;
    pcl::PointCloud<PointT>::Ptr filtered_cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    QMutex _mtx;
    QMutex viewer_mtx;
    boost::signals2::connection connection_kinect; 
    int nframe; 
    QTimer *viewer_timer; 

    //Voxel variables
    pcl::VoxelGrid<PointT> vox;
    const float leaf_size;
    
    //Passtrhough variables
    pcl::PassThrough<PointT> pass;
    float min_Z; float v_max_Z;
   
    //Transformation variables
    float v_theta;
    float cosTheta;
    float sinTheta;

    //RANSAC variables
    int v_n_iter; 
    float v_dist, v_perc_trash;
    PlaneT floor;
    std::list<PlaneT> planes;
    
    //Routing variables
    int Map[N][M];
    float delta_x;
    float delta_z;

public slots:
    //Methods
    //void cloudViewer_init ();
    void Voxelgrid_filter();
    void PassThrough_Filter();
    void Transformation_cloud();
    void update_Map();
    void update_Graphics();
    void Routing();
    void InitMap();
    void update_Path();
    void Ransac();
    void Get_Angle();
    void print_Map();
    //void on_setParam_clicked();
    void on_max_Z_valueChanged();
    void on_encum_valueChanged();
    void on_n_iter_valueChanged();
    void on_perc_trash_valueChanged();
    void on_distance_valueChanged();
    void on_theta_valueChanged();
    //void view_Planes(std::list<PlaneT> planes, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    void on_Start_clicked();
    void on_offset_valueChanged();
    void on_pt2fw_valueChanged();
    
private slots:
    void timeoutSlot();
};


#endif // MAINWINDOW_H
