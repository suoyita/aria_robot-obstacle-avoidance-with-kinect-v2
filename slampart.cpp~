#include "slam.h"

float voxel_resolution = 0.008f;
float seed_resolution = 0.1f;

float posdist = 1.5;
float negdist = -1.5;
int min_clusters_contains = 3;

g2o::RobustKernel* robustKernel = g2o::RobustKernelFactory::instance()->construct( "Cauchy" );

cv::Point3f point2dTo3d (cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera) {
    cv::Point3f p; // 3D 点
    p.z = double( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}

PointCloudT::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloudT::Ptr cloud ( new PointCloudT );

    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

bool CompareVector(const PointT &pt1, const PointT &pt2) {
    //求向量的模
    float m1 = sqrt((float)(pt1.x * pt1.x + pt1.z * pt1.z));
    float m2 = sqrt((float)(pt2.x * pt2.x + pt2.z * pt2.z));
    //两个向量分别与(1, 0)求内积
    float v1 = pt1.x / m1, v2 = pt2.x / m2;
    return (v1 > v2 || (v1 == v2 && m1 < m2));
}

PTARRAY ConvexHull(PointCloudT &cluster){
    PTARRAY vecSrc, vecCH;
    for (int j = 0; j < cluster.size(); ++j) {
        PointT point = cluster.points[j];
        point.y = 0;
        vecSrc.push_back(point);
    }
    PointT ptBase = vecSrc.front(); //将第1个点预设为最小点
    for (PTARRAY::iterator i = vecSrc.begin() + 1; i != vecSrc.end(); ++i) {
        //如果当前点的y值小于最小点，或y值相等，x值较小
        if (i->z < ptBase.z || (i->z == ptBase.z && i->x > ptBase.x)) {
            //将当前点作为最小点
            ptBase = *i;
        }
    }
    //计算出各点与基点构成的向量
    for (PTARRAY::iterator i = vecSrc.begin(); i != vecSrc.end();) {
        //排除与基点相同的点，避免后面的排序计算中出现除0错误
        if ((i->x == ptBase.x) && (i->z == ptBase.z)) {
            i = vecSrc.erase(i);
        }
        else {
            //方向由基点到目标点
            i->x -= ptBase.x, i->z -= ptBase.z;
            ++i;
        }
    }
    //按各向量与横坐标之间的夹角排序
    sort(vecSrc.begin(), vecSrc.end(), &CompareVector);
    //删除相同的向量,unique == yoiwenti

//    vecSrc.end() = unique(vecSrc.begin(), vecSrc.end());
    //计算得到首尾依次相联的向量
    for (PTARRAY::reverse_iterator ri = vecSrc.rbegin();
        ri != vecSrc.rend() - 1; ++ri) {
        PTARRAY::reverse_iterator riNext = ri + 1;
        //向量三角形计算公式
        ri->x -= riNext->x, ri->z -= riNext->z;
    }
    //依次删除不在凸包上的向量
    for (PTARRAY::iterator i = vecSrc.begin() + 1; i != vecSrc.end(); ++i) {
        //回溯删除旋转方向相反的向量，使用外积判断旋转方向
        for (PTARRAY::iterator iLast = i - 1; iLast != vecSrc.begin();) {
            float v1 = i->x * iLast->z, v2 = i->z * iLast->x;
            //如果叉积小于0，则无没有逆向旋转
            //如果叉积等于0，还需判断方向是否相逆
            if (v1 < v2 || (v1 == v2 && i->x * iLast->x > 0 &&
                i->z * iLast->z > 0)) {
                    break;
            }
            //删除前一个向量后，需更新当前向量，与前面的向量首尾相连
            //向量三角形计算公式
            i->x += iLast->x, i->z += iLast->z;
            iLast = (i = vecSrc.erase(iLast)) - 1;
        }
    }
    //将所有首尾相连的向量依次累加，换算成坐标
    vecSrc.front().x += ptBase.x, vecSrc.front().z += ptBase.z;
    for (PTARRAY::iterator i = vecSrc.begin() + 1; i != vecSrc.end(); ++i) {
        i->x += (i - 1)->x, i->z += (i - 1)->z;
    }
    //添加基点，全部的凸包计算完成
    vecSrc.push_back(ptBase);

    return vecSrc;
}

#if ShowEdgeResult
void drawEdge(vector<PTARRAY> &final_edges, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud, bool hold,
                         boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer) {
#else
void drawEdge(vector<PTARRAY> &final_edges, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud, bool hold) {
#endif
printf("I am in\n");
    bool disable_transform = false;

    struct timeval tt1, tt2, tt3;

    float color_importance = 0.2f;
    float spatial_importance = 0.4f;
    float normal_importance = 1.0f;

    //////////////////////////////  //////////////////////////////
    ////// This is how to use supervoxels
    //////////////////////////////  //////////////////////////////
    gettimeofday(&tt1, NULL);
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int i = 0; i < origin_cloud->size(); i++) {
        if (origin_cloud->points[i].y > negdist && origin_cloud->points[i].y < posdist)
            cloud->push_back(origin_cloud->points[i]);
    } 

    pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
    if (disable_transform)
        super.setUseSingleCameraTransform (false);
    super.setInputCloud (cloud);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);

    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

    pcl::console::print_highlight ("Extracting supervoxels!\n");
    super.extract (supervoxel_clusters);
    pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //viewer->setBackgroundColor (0, 0, 0);
#if ShowEdgeResult
    PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
    viewer->addPointCloud (voxel_centroid_cloud, "raw centroids");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "raw centroids");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "raw centroids");

    PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
    viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");
#endif

    //PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
    
    //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
    //viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

    //std::vector< std::pair<PointTA,PointTA> > edge_lines;
    
    pcl::console::print_highlight ("Getting supervoxel adjacency\n");

    
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency (supervoxel_adjacency);

#if USE_SUPERVOXEL_CLUSTERS
    int max_label = super.getMaxLabel();
    printf("Max label %d\n", super.getMaxLabel());

    int cluster_id = 1, current_label;
    int *belong = new int[max_label+1];
    memset(belong, 0, sizeof(int)*(max_label+1));
#endif

    uint32_t supervoxel_label, adjacent_label;
    //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
    std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
    for ( ; label_itr != supervoxel_adjacency.end (); ) {
        //First get the label
        supervoxel_label = label_itr->first;
#if USE_SUPERVOXEL_CLUSTERS
        if (belong[supervoxel_label] == 0)
            belong[supervoxel_label] = cluster_id++;
        current_label = belong[supervoxel_label];
#endif

        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        PointCloudTA adjacent_supervoxel_centers; // RGBA
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr) {
#if USE_SUPERVOXEL_CLUSTERS
            adjacent_label = adjacent_itr->second;
            if (belong[adjacent_label] == 0)
                belong[adjacent_label] = current_label;
            else if (belong[adjacent_label] != current_label) {
                for (int i = 0; i < max_label+1; i++) {
                    if (belong[i] == belong[adjacent_label])
                        belong[i] = current_label;
                }
            }
#endif

            pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_label);
            adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
        }

        PointTA center = supervoxel->centroid_;
        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;
        /*if (hold) {
            std::stringstream ss;
            ss << "supervoxel_" << supervoxel_label;
            addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
        }*/
/*#if ! USE_SUPERVOXEL_CLUSTERS
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
        vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
        vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

         //if (center.y > negdist && center.y < posdist) {
         if (center.y > negdist && center.y < posdist) { //} inAera(center.x, center.y, t, negdist, posdist)){
            center.y = 0;
            PointCloudTA::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
            for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr) {
                PointTA neighbor = *reinterpret_cast<PointTA*>(adjacent_itr->data);
                // if (neighbor.y >negdist && neighbor.y < posdist) {
                if (neighbor.y > negdist && neighbor.y < posdist) { //inAera(neighbor.x, neighbor.y, t, negdist, posdist)){
                    neighbor.y = 0;
                    edge_lines.push_back( std::pair<PointTA,PointTA>(center, neighbor) );
                    points->InsertNextPoint(center.data);
                    points->InsertNextPoint(neighbor.data);
                }
            }
        }

        // Create a polydata to store everything in
        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
        // Add the points to the dataset
        polyData->SetPoints (points);
        polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
        for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
            polyLine->GetPointIds ()->SetId (i,i);
        cells->InsertNextCell (polyLine);
        // Add the lines to the dataset
        polyData->SetLines (cells);
        viewer->addModelFromPolyData (polyData, ss.str());
#endif*/
        //Move iterator forward to next label
        label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }

#if USE_SUPERVOXEL_CLUSTERS
    /*for (int i = 0; i < max_label+1; i++) {
        printf("(%d:%d) ", i, belong[i]);
        if (!(i%50)) printf("\n");
    }
    printf("\n");*/

    PointCloudT sum;
    std::vector< std::pair<int,PointCloudT> > clusters;
    int cells_count;
    char clustername[20];
    for (int i = 1; i < cluster_id; i++) {
        printf("iter %d\n", i);
        cells_count = 0;
        PointCloudT single_cluster;
        PointCloudTA::Ptr labeled_centroids;
        for (int j = 1; j < max_label+1; j++) {
            if (belong[j] == i) {
                cells_count ++;
                single_cluster += *((supervoxel_clusters.at(j))->voxels_);
                //labeled_centroids->push_back( (supervoxel_clusters.at(j))->centroid_ );
            }
        }
        if (cells_count > min_clusters_contains) {
            sum += single_cluster; // TODO: sum zuihouyeyaofanhui
            clusters.push_back( std::pair<int,PointCloudT>(cells_count, single_cluster) );


            //sprintf(clustername, "cluster_%d", i);
            //viewer->addPointCloud (labeled_centroids, clustername);
            //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,4.0, clustername);
            //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, clustername);
        }
    }
    gettimeofday(&tt2, NULL);
    printf("Time used for cluster = %lf\n", gettime(tt1, tt2));
    final_edges.clear();//vector<PTARRAY> final_edges;
    for (int i = 0; i < clusters.size(); i++) {
        PointCloudT cluster = clusters[i].second;
        final_edges.push_back( ConvexHull(cluster) );
    }
    gettimeofday(&tt3, NULL);
    printf("Time used for Convex = %lf\n", gettime(tt2, tt3));

#if ShowEdgeResult
    char final_edge_name[20];
    for (int i = 0; i < final_edges.size(); i++) {
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
        vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
        vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

#if EdgeDebug
        char debug_name[20];
        sprintf(debug_name, "EdgeViewer_%d_debug", i);
        boost::shared_ptr<pcl::visualization::PCLVisualizer> edge_viewer_debug(new pcl::visualization::PCLVisualizer(debug_name));
        edge_viewer_debug->setBackgroundColor (0, 0, 0);
        sprintf(debug_name, "cloud_%d_debug", i);
        edge_viewer_debug->addPointCloud(cloud, debug_name);

        edge_viewer_debug->addPointCloud (voxel_centroid_cloud, "raw centroids");
        edge_viewer_debug->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "raw centroids");
        edge_viewer_debug->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "raw centroids");

        edge_viewer_debug->addPointCloud (labeled_voxel_cloud, "labeled voxels");
        edge_viewer_debug->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

        vtkSmartPointer<vtkPoints> points_debug = vtkSmartPointer<vtkPoints>::New ();
        vtkSmartPointer<vtkCellArray> cells_debug = vtkSmartPointer<vtkCellArray>::New ();
        vtkSmartPointer<vtkPolyLine> polyLine_debug = vtkSmartPointer<vtkPolyLine>::New ();
#endif

        final_edges[i][0].y = 0;
        for (int j = 0; j < final_edges[i].size() - 1; j++) {
            final_edges[i][j+1].y = 0;
            points->InsertNextPoint(final_edges[i][j].data);
            points->InsertNextPoint(final_edges[i][j+1].data);

#if EdgeDebug
            points_debug->InsertNextPoint(final_edges[i][j].data);
            points_debug->InsertNextPoint(final_edges[i][j+1].data);
#endif      
        }
        points->InsertNextPoint(final_edges[i][final_edges[i].size()-1].data);
        points->InsertNextPoint(final_edges[i][0].data);

        // Create a polydata to store everything in
        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
        // Add the points to the dataset
        polyData->SetPoints (points);
        polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
        for(unsigned int j= 0; j < points->GetNumberOfPoints (); j++)
            polyLine->GetPointIds ()->SetId (j,j);
        cells->InsertNextCell (polyLine);
        // Add the lines to the dataset
        polyData->SetLines (cells);
        sprintf(final_edge_name, "final_edge_%d", i);
        viewer->addModelFromPolyData (polyData, final_edge_name);

#if EdgeDebug
        points_debug->InsertNextPoint(final_edges[i][final_edges[i].size()-1].data);
        points_debug->InsertNextPoint(final_edges[i][0].data);

        vtkSmartPointer<vtkPolyData> polyData_debug = vtkSmartPointer<vtkPolyData>::New ();
        // Add the points to the dataset
        polyData_debug->SetPoints (points_debug);
        polyLine_debug->GetPointIds  ()->SetNumberOfIds(points_debug->GetNumberOfPoints ());
        for(unsigned int j = 0; j < points_debug->GetNumberOfPoints (); j++)
            polyLine_debug->GetPointIds ()->SetId (j,j);
        cells_debug->InsertNextCell (polyLine_debug);
        // Add the lines to the dataset
        polyData_debug->SetLines (cells_debug);
        sprintf(debug_name, "edges_%d_debug", i);
        edge_viewer_debug->addModelFromPolyData (polyData_debug, debug_name);
       // while (!edge_viewer_debug->wasStopped());
#endif
    }
#endif
#endif

    return; // TODO: if USE_SUPERVOXEL_CLUSTERS, an empty vector would be returned
}

Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec ) {
    cv::Mat R, M;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    //Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
    //cvInvert(T,T2,CV_SVD);

    Eigen::AngleAxisd angle(r);
    Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(0,1); 
    T(2,3) = tvec.at<double>(0,2);


    return T;//.inverse();
}

double normofTransform(cv::Mat rvec, cv::Mat tvec) {
    double a = cv::norm(rvec), b = 2*M_PI-cv::norm(rvec);
    return fabs(a<=b?a:b) + fabs(cv::norm(tvec));
}

void* edge_event(void* args) {
    ImageDB *imgdb = (ImageDB*)args;
    ImageList *current = imgdb->imglists[imgdb->current_id];

#if ShowEdgeResult
    boost::shared_ptr<pcl::visualization::PCLVisualizer> edge_viewer (new pcl::visualization::PCLVisualizer ("EdgeViewer"));
    edge_viewer->setBackgroundColor (0, 0, 0);
    edge_viewer->addPointCloud (current->cloud, "base cloud");
    drawEdge(current->edge, current->cloud, 1, edge_viewer);
    while (!edge_viewer->wasStopped()) {
        edge_viewer->spinOnce(100);
    }
#else
    drawEdge(current->edge, current->cloud, 1);
#endif

    printf("finished egde detect %d\n", current->id);

    if (imgdb->local_viewer) {
        char final_edge_name[20];
        sprintf(final_edge_name, "cloud_%d", current->id);

        imgdb->local_viewer->removeAllShapes();
        imgdb->local_viewer->removeAllPointClouds();
        imgdb->local_viewer->addPointCloud(current->cloud, final_edge_name);
        //imgdb->local_viewer->spinOnce(100);
        
        for (int i = 0; i < current->edge.size(); i++) {
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
            vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
            vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();
            for (int j = 0; j < current->edge[i].size()-1; j++) {
                points->InsertNextPoint(current->edge[i][j].data);
                points->InsertNextPoint(current->edge[i][j+1].data);
            }
            points->InsertNextPoint(current->edge[i][current->edge[i].size()-1].data);
            points->InsertNextPoint(current->edge[i][0].data);

            vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
            // Add the points to the dataset
            polyData->SetPoints (points);
            polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
            for(unsigned int j= 0; j < points->GetNumberOfPoints (); j++)
                polyLine->GetPointIds ()->SetId (j,j);
            cells->InsertNextCell (polyLine);
            // Add the lines to the dataset
            polyData->SetLines (cells);
            sprintf(final_edge_name, "final_edge_%d", i);
            imgdb->local_viewer->addModelFromPolyData (polyData, final_edge_name);
        }
        //while (!imgdb->local_viewer->wasStopped()) 
imgdb->local_viewer->spinOnce(100);
        //imgdb->local_viewer->resetStoppedFlag();
        printf("+========  %d ========\n===============\n", current->id);
    }


    imgdb->edge_thread_in_use = false;
    if (!imgdb->qby_thread_in_use) imgdb->edge_thread_item = current;

    return NULL;
}

void* concate_event(void* args) {
    ImageDB *imgdb = (ImageDB*)args;
    ImageList *current = imgdb->imglists[imgdb->current_id];

    // do nothing

    return NULL;
}

double gettime(struct timeval &tt1, struct timeval &tt2) {
    return tt2.tv_sec - tt1.tv_sec + (double)(tt2.tv_usec - tt1.tv_usec) / 1000000;
}


