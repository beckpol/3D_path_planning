#include "mainwindow.h"

#include <QApplication>
#include <QEvent>
#include <QObject>

#include <vtkRenderWindow.h>

void MainWindow::callback_cloud (const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
    QMutexLocker locker (&_mtx);
    get_cloud = cloud;
    nframe++;
    processing();
}

void MainWindow::processing()
{ 
  nopoints = false;
  if (get_cloud && capture)
    { 
      Voxelgrid_filter();
      PassThrough_Filter();
      if(verbose){
         cout << "--------------------------" << endl;
         cout << "nframe: " << nframe << endl;
         cout << "get_cloud points:" << get_cloud->points.size() << endl;
         cout << "filtered_points: " << filtered_cloud->points.size() << endl;
      }
      if(filtered_cloud->points.size()>100){ 
         Transformation_cloud();
         Ransac();
         InitMap();
         update_Map();
         Routing();
         update_Path();
         Get_Angle();
         if(verbose){
            std::cout << "planes found = " << planes.size() << std::endl;
            cout << "points_floor: " << floor.points->points.size() << endl;
            print_Map();
            cout << "angle " << angle << endl;
         }  
      }else{
         InitMap();
         angle = 90;
         nopoints = true;
      }
    }
}

void MainWindow::on_Start_clicked(){
    capture = !capture;
}

void MainWindow::Voxelgrid_filter()
{
  pcl::PointCloud<PointT> cloud_filtered;
  {
  //QMutexLocker locker (&_mtx);
  vox.setInputCloud (get_cloud);
  vox.setLeafSize (leaf_size,leaf_size,leaf_size);
  vox.filter (cloud_filtered);
  }
  *filtered_cloud = cloud_filtered;
}

void MainWindow::PassThrough_Filter()
{
  pcl::PointCloud<PointT> cloud_filtered;
  pass.setFilterFieldName ("z");
  pass.setInputCloud (filtered_cloud);
  pass.setFilterLimits (min_Z,v_max_Z);
  pass.filter (cloud_filtered);
  *filtered_cloud = cloud_filtered;
}

void MainWindow::Transformation_cloud()
{
  pcl::PointCloud<PointT> transformated_cloud;
  
  for(pcl::PointCloud<PointT>::const_iterator it = filtered_cloud->begin();
  it != filtered_cloud->end(); ++it)
  {
    PointT point;
    point.x = it->x;
    point.y = (it->y)*cosTheta - (it->z)*sinTheta;
    point.z = (it->y)*sinTheta + (it->z)*cosTheta;
    transformated_cloud.push_back(point);
   }
   
   *filtered_cloud=transformated_cloud; 
}

void MainWindow::Ransac()
{
  planes.clear();
  pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointT> extract;
  pcl::SACSegmentation<PointT> seg;
  
  int nr_point = (int) filtered_cloud->points.size();
  
  seg.setOptimizeCoefficients(true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (v_n_iter);
  seg.setDistanceThreshold (v_dist);

  float max = 0.0;
  while(filtered_cloud->points.size() > v_perc_trash*nr_point)
  {
    PlaneT current_plane;
    pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
   
    seg.setInputCloud(filtered_cloud);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
    }
    
    extract.setInputCloud (filtered_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_in);
    current_plane.points = cloud_in;
    current_plane.coefficients = coefficients;
    
    extract.setNegative (true);
    extract.filter (*cloud_out);
    filtered_cloud.swap(cloud_out);
   
    PointT maxPlane(0,0,0);
    PointT minPlane(0,0,0);
    pcl::getMinMax3D(*(current_plane.points),minPlane,maxPlane);
    
    float mean = (maxPlane.y+minPlane.y)/2;
    if (mean > max){
       max = mean;
       planes.push_front(current_plane);
    }else{
       planes.push_back(current_plane);
    }
  }
  floor = planes.front();
  data_modified = 1;
}


MainWindow::MainWindow(QWidget *parent) :
    ui(new Ui::MainWindow), 
    leaf_size (0.03f),
    filtered_cloud(new pcl::PointCloud<PointT>)
{
    verbose=0;
    
    viewer_timer = new QTimer (this);
    viewer_timer->start (5);
    connect (viewer_timer, SIGNAL (timeout ()), this, SLOT (timeoutSlot ()));
    
    ui->setupUi(this);

    this->setWindowTitle ("kinect indoor navigation");
    viewer.reset (new pcl::visualization::PCLVisualizer ("", false));
    viewer->setCameraPosition(0,-1,-4,0,-1,0,0);
    ui->qvtk_widget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtk_widget->GetInteractor (), ui->qvtk_widget->GetRenderWindow ());
    viewer->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    ui->qvtk_widget->update (); 
    
    kinect = new pcl::OpenNIGrabber ();
    boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> cb = boost::bind (&MainWindow::callback_cloud, this, _1);
    connection_kinect = kinect->registerCallback (cb);
    nframe=1;
   
    arrowScene = new QGraphicsScene(this);
    ui->arrowView->setScene(arrowScene);

    QBrush greenBrush(Qt::green);
    QPen blackpen(Qt::black);
    blackpen.setWidth(6);
    
    QPolygonF arrow;
    arrow.append(QPoint(10,50));
    arrow.append(QPoint(10,-40));
    arrow.append(QPoint(20,-40));
    arrow.append(QPoint(0,-70));
    arrow.append(QPoint(-20,-40));
    arrow.append(QPoint(-10,-40));
    arrow.append(QPoint(-10,50));
    arrow.append(QPoint(10,50));
    p_arrow = arrowScene->addPolygon(arrow,blackpen,greenBrush);
    
    InitMap();
    
    // default values
    data_modified = 0;
    cur_angle = 0.0f;
    delta_x = 3.0/N; //3.0 m is the size of x dimension Map
    delta_z = 3.0/M; //3.0 m is the size of z dimension Map
    min_Z = 0.5f;
    v_pt2fw = ui->pt2fw->value();
    v_max_Z = ui->max_Z->value();
    v_offset = ui->offset->value();
    v_dist = ui->distance->value();
    v_perc_trash = ui->perc_trash->value()/100;
    v_n_iter = ui->n_iter->value();
    v_encum = ceil((ui->encum->value()/delta_x));
    v_theta = -(ui->theta->value()/180)*PI;
    cosTheta = cos(v_theta);
    sinTheta = sin(v_theta);
    capture = false;
    kinect->start();
}

void MainWindow::on_distance_valueChanged()
{
    v_dist = ui->distance->value();
    statusBar()->showMessage(tr("Distance changed!"));
}

void MainWindow::on_theta_valueChanged()
{
    v_theta = -(ui->theta->value()/180)*PI;
    cosTheta = cos(v_theta);
    sinTheta = sin(v_theta);
    statusBar()->showMessage(tr("Theta changed!"));
}

void MainWindow::on_offset_valueChanged()
{
    v_offset = ui->offset->value();
    statusBar()->showMessage(tr("Offset changed!"));
}

void MainWindow::on_pt2fw_valueChanged()
{
    v_pt2fw = ui->pt2fw->value();
    statusBar()->showMessage(tr("point to follow changed!"));
}

void MainWindow::on_max_Z_valueChanged()
{
    v_max_Z = ui->max_Z->value();
    statusBar()->showMessage(tr("Max_Z changed!"));
}

void MainWindow::on_perc_trash_valueChanged()
{
    v_perc_trash = ui->perc_trash->value()/100;
    statusBar()->showMessage(tr("Percentage trashed changed!"));
}

void MainWindow::on_n_iter_valueChanged()
{
    v_n_iter = ui->n_iter->value();
    statusBar()->showMessage(tr("Number of iterations changed!"));
}

void MainWindow::on_encum_valueChanged()
{
     v_encum = ceil((ui->encum->value()/delta_x));
    statusBar()->showMessage(tr("Encumbrance changed!"));
} 

void MainWindow::InitMap()
{
  path = NULL;
  
  for(int i=0;i<K;i++){
    zones[i] = NULL;
  }
  
  for(int i=0;i<N;i++){
    for(int j=0;j<M;j++){
       Map[i][j]=0;
    }
  }
}

void MainWindow::update_Map()
{
  int i,j;
  for(pcl::PointCloud<PointT>::const_iterator it = floor.points->begin();
  it != floor.points->end(); ++it)
  {
    i=(int)((it->z-v_offset)/delta_z);
    j=(int)((it->x)/(delta_x)+(M/2));
    if(i<=N-1 && i>=0 && j<=M-1 && j>=0)
    {
       Map[i][j]=1;
    }
  }
}

void MainWindow::Routing()    //dz e encumbrance 2/3/5
{
    int dz = N/K;
    bool found = 0;
    ZoneT *id=NULL, *id_path=NULL,*id_zone=NULL;

    for(int i=0;i<N;i+=dz){
       int prev=-1;
       int next=-1;
       for(int j=0;j<M;j++){
         bool check = 1;
         for(int w=0;w<dz;w++){
             if(Map[i+w][j]==0){
                 check=0;
                 break;
             }
         }
         if(check==1){
             next=j;
             if(next>=prev+v_encum && next==M-1){
                 zones[i/dz]=Insert_zone(zones[i/dz],prev+1,next,i,dz);
             }
         }else{
             if(next>=prev+v_encum){
                 zones[i/dz]=Insert_zone(zones[i/dz],prev+1,next,i,dz);
             }
         prev=j;
         }
       }
    }

    int t=0;
    while(t<K && found==0){
       id_zone=zones[K-1-t];
       while(id_zone!=NULL && found==0){
          int j=1;
          path = new ZoneT(id_zone->datap,id_zone->datan,id_zone->id_z,id_zone->dim_z,NULL);
          id_path=path;
          while(j<K-t){
              id=zones[K-1-t-j];
              bool check = 0;
              while(id!=NULL && check==0){
                  int maxp,minn;
                  if(id_path->datap>id->datap){
                     maxp=id_path->datap;
                  }else{
                     maxp=id->datap;
                  }
                  if(id_path->datan<id->datan){
                     minn=id_path->datan;
                  }else{
                     minn=id->datan;
                  }
                  if(minn-maxp+1>=v_encum){
                     id_path->next = new ZoneT(id->datap,id->datan,id->id_z,id->dim_z,NULL);
                     id_path=id_path->next;
                     check=1;
                  }
                  if(check==0){
                     id=id->next;
                  }
              }
              if(id==NULL){
                 break;
              }
              if((K-1-t-j)==0){
                 found=1;
              }
              j++;
          }
          if(found==0){
             id_zone=id_zone->next;
             path=NULL;
          }
       }
       t++;
    }
}

ZoneT* MainWindow::Insert_zone(ZoneT *zone, int prev, int next, int id, int dim)
{
  ZoneT *idx=zone;

  if(idx==NULL){
    idx = new ZoneT(prev,next,id,dim,NULL);
    return idx;
  }
  if((next-prev)>=(idx->datan-idx->datap)){
    idx = new ZoneT(prev,next,id,dim,idx);
    return idx;
  }

  while(idx->next!=NULL && (idx->next->datan-idx->next->datap)>(next-prev)){
        idx=idx->next;
  }
  idx->next = new ZoneT(prev,next,id,dim,idx->next);
  return zone;
}

void MainWindow::update_Path()
{
   ZoneT * id=NULL;
   id=path;
   while(id!=NULL){
      Map[id->id_z+((id->dim_z-1)/2)][id->med]=2;
      id=id->next;
   }
}

void MainWindow::Get_Angle()
{
    ZoneT* old=path;
    ZoneT* id_new=NULL, *id_oldp=path, *id_oldn=path;
    path=NULL;
    //bool found=false;

    if(old==NULL){
      angle = 90.0;

    }else{

       while(old->next!=NULL){
          id_oldn=old;
          id_oldp=old;
          while(id_oldn->next!=NULL){
              id_oldp = id_oldn;
              id_oldn = id_oldn->next;
          }
        if(path==NULL){
           path=id_oldn;
           id_new=path;
           id_oldp->next=NULL;
        }else{
           id_new->next = id_oldn;
           id_oldp->next = NULL;
           id_new=id_new->next;
        }
    }
    id_new->next = old;
    old=NULL;

    id_new=path;
    int i=0;
    while(id_new->next!=NULL && i<v_pt2fw-1){
        id_new = id_new->next;
        i++;
    }
    angle = atan(((id_new->med-14)*delta_x)/(v_offset+delta_z*id_new->id_z+0.15));
    angle = 180*angle/PI;
    }
}

void MainWindow::update_Graphics()
{
    QBrush blueBrush(Qt::blue);
    QBrush redBrush(Qt::red);
    QBrush greenBrush(Qt::green);
    QPen blackpen(Qt::black);
    
    ui->arrowView->rotate(angle-cur_angle);
    cur_angle = angle;
  
    radarScene = new QGraphicsScene(this);
    ui->radarView->setScene(radarScene);

    for(int i=0;i<N;i++){
        for(int j=0;j<M;j++){
            QRect rec(0+6*j, 0+6*(N-1-i), 6,6);
            blackpen.setWidth(0);
            switch(Map[i][j]){
            case 1:
               p_rec = radarScene->addRect(rec,blackpen,greenBrush);
               break;
            case 0:
               p_rec = radarScene->addRect(rec,blackpen,blueBrush);
               break;
            case 2:
               p_rec = radarScene->addRect(rec,blackpen,redBrush);
               break;
            }
        }
    }
}

void MainWindow::timeoutSlot(){
   QMutexLocker locker (&_mtx);
   if(data_modified==1){
      char name[40];
      unsigned int red = 0; 
      unsigned int grn = 255;
      unsigned int blu = 0;
      viewer->removeAllPointClouds();
   
      int i=0;
      for(std::list<PlaneT>::iterator it=planes.begin(); it!=planes.end(); ++it){
         sprintf (name,"plane_%d",i);
         if(i>0){
            red = rand()%256;
            grn = rand()%100;
            blu = rand()%256;
         }
         pcl::visualization::PointCloudColorHandlerCustom <PointT> color (it->points,red,grn,blu);
         viewer->addPointCloud (it->points,color,name);
         viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,name);  
         i++; 
      }
   }
   ui->qvtk_widget->update();
   data_modified = 0;
   
   update_Graphics();
}

void MainWindow::print_Map()
{
  std::cout << "------------------------------------------------------------" << std::endl;
   for(int i=0;i<N;i++){
     for(int j=0;j<M;j++){
       std::cout << "|" << Map[N-i-1][j];
     }
     std::cout << "|" << std::endl;
   }
  std::cout << "------------------------------------------------------------" << std::endl;
}

