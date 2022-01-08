#include <iostream>
#include <memory>
#include <thread>
#include <math.h>

#include <chrono>
#include <numeric> //std::iota
#include <omp.h>

#include "open3d/Open3D.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "../include/polyhedron.h"
#include "../include/utils.h"



//global to allow render thread to finish when told to
bool finished = false;

class ScanPoint{
    public:    
        static std::shared_ptr<open3d::geometry::PointCloud> pointCloud;
        static double ang_check;
        static double standoff;

        std::vector<int> viewPoints;
        std::vector<int> validPoints;
        std::vector<int> unvisitedPoints;
        double ratio;
        int centrePointIdx;
        Eigen::Vector3d centreNormal;
        Eigen::Vector3d centrePoint;
        Eigen::Vector3d centreOffset;

        bool used;
        Eigen::Vector3d sensorPosition;
        Eigen::Vector3d sensorNormal;

    void addViewPoint(int pt) {
        viewPoints.push_back(pt);
    }
    void getRatio(){
        if (viewPoints.size()>0){
            ratio = (double) validPoints.size()/ (double) viewPoints.size();
        }
        else{
            ratio = 0.0;
        }
    }
    void getValidPoints() {
        validPoints.clear();
        for (std::vector<int>::iterator pt = viewPoints.begin(); pt != viewPoints.end();++pt){
            if (pointCloud->normals_[*pt].dot(centreNormal)>=ang_check){
                validPoints.push_back(*pt);
                unvisitedPoints.push_back(*pt);
            }
        }
    }

    ScanPoint(int cPtIdx, std::vector<int> viewPts) {
        centrePointIdx = cPtIdx;
        viewPoints = viewPts;
        centrePoint = pointCloud->points_[cPtIdx];
        centreNormal = pointCloud->normals_[cPtIdx];
        used = false;
        
    }

    ScanPoint(int cPtIdx) {
        centrePointIdx = cPtIdx;
        centrePoint = pointCloud->points_[cPtIdx];
        centreNormal = pointCloud->normals_[cPtIdx];
        sensorPosition = centrePoint+standoff*centreNormal;
        sensorNormal = -centreNormal;
        used = false;
        viewPoints.reserve(500);
        validPoints.reserve(500);
    }

    bool operator > (const ScanPoint& scPt) const
    {
        //used to be ratio but doesn't take into account number of points in dof
        return (unvisitedPoints.size() > scPt.unvisitedPoints.size());
    }

};

std::shared_ptr<open3d::geometry::PointCloud> ScanPoint::pointCloud = std::make_shared<open3d::geometry::PointCloud>();
double ScanPoint::ang_check = 0;
double ScanPoint::standoff = 0;



void doRender(std::shared_ptr<open3d::geometry::PointCloud> pcd, std::shared_ptr<open3d::geometry::TriangleMesh> mesh_cone, std::shared_ptr<open3d::geometry::TriangleMesh> mesh_cyl) {
    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow();
    vis.AddGeometry(pcd);
    vis.AddGeometry(mesh_cone);
    vis.AddGeometry(mesh_cyl);
    while (!finished) {
            vis.UpdateGeometry(pcd);
            vis.UpdateGeometry(mesh_cone);
            vis.UpdateGeometry(mesh_cyl);
            vis.PollEvents();
            vis.UpdateRender();
    }
}



int main() {

    const Eigen::Vector3d z_axis = Eigen::Vector3d(0,0,1);
    
    // Sensor Variables //

    /* Depth of field - allowable deviation from surface that will still be in focus
                        Build a box of this width and test all points within */
    double dof = 0.02;

    /* Standoff distance - distance sensor is from surface, height of pyramid protruding from surface
                    Check if no points within pyramid to obstruct view    */
    double standoff = 0.6;

    // Field of view - width of square located by image
    double fov = 0.4;

    // Allowable angle deviation (in degrees)
    double angle_dev = 30.0;
    const double ang_check = std::cos(angle_dev*M_PI/180);

    //geometry approximating the sensor's frustrum
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh_cyl = open3d::geometry::TriangleMesh::CreateBox(fov,fov,dof);
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh_cone = CreateRectPyramid(fov, fov,standoff);
    //relocate sensor geometry to align
    mesh_cone->Translate(z_axis*dof/2.0);
    mesh_cyl->Translate(Eigen::Vector3d(-fov/2.0,-fov/2.0,-dof/2.0));
    mesh_cyl->PaintUniformColor(Eigen::Vector3d(0.5,0,0.5));

    open3d::io::ReadPointCloud("../../../../box_sampled_n.ply", *ScanPoint::pointCloud);

    ScanPoint::ang_check = ang_check;
    ScanPoint::standoff = standoff;

    ScanPoint::pointCloud->PaintUniformColor(Eigen::Vector3d(1,0,0));

    //render visualiser on another thread
    std::thread render(doRender,ScanPoint::pointCloud, mesh_cone, mesh_cyl);


    //transformation matrix used to move sensor meshes about
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    //cross and dot products of the z axis and the current sensor normal
    Eigen::Vector3d v;
    double c;

    //polyhedron - lightweight point in mesh check
    Polyhedron poly_cyl = Polyhedron(mesh_cyl->triangles_, mesh_cyl->vertices_);
  
    open3d::geometry::KDTreeFlann pcd_tree(*ScanPoint::pointCloud);

    //octomap used to check rays from surface to sensor are not obstructed
    std::shared_ptr<octomap::Pointcloud> octoPcd = std::make_shared<octomap::Pointcloud>();
    for (auto& pt:ScanPoint::pointCloud->points_) {
        //Move from open3d pointcloud to octomap pcl
        octoPcd->push_back(pt[0],pt[1],pt[2]);
    }
    const octomap::point3d origin = octomap::point3d(0.0,0.0,0.0);
    std::shared_ptr<octomap::OcTree> tree = std::make_shared<octomap::OcTree>(0.01);
    tree->insertPointCloud(*octoPcd,origin);
    octomap::KeyRay rays;

    //initially downsample pointcloud to get a small list of points to sample which are evenly spaced
    double voxelSize = 0.3;
    std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, Eigen::MatrixXi, std::vector<std::vector<int>>> downTuple = ScanPoint::pointCloud->VoxelDownSampleAndTrace(voxelSize,ScanPoint::pointCloud->GetMinBound(),ScanPoint::pointCloud->GetMaxBound());
    
    std::vector<int> testList;

    for (int i = 0;i<std::get<0>(downTuple)->points_.size();++i){
        std::vector<int> closestPt;
        std::vector<double> distance2;
        pcd_tree.SearchKNN(std::get<0>(downTuple)->points_[i],1,closestPt,distance2);
        ScanPoint::pointCloud->colors_[closestPt[0]] = Eigen::Vector3d(0,0,1);
        testList.push_back(closestPt[0]);
    }

    std::vector<ScanPoint> ScannedPoints;
    //ScannedPoints.reserve(1000);

    std::vector<int> neighbours;
    std::vector<double> neighbourDist;
    
    //vector of indexes to pointcloud pts - pts removed when visited 
    std::vector<int> unvisited(ScanPoint::pointCloud->points_.size());
    std::iota(unvisited.begin(),unvisited.end(),0);

    int count = 0;

    //loop until every point is visited
    while (unvisited.size()>0){
        // testList contains points that are (hopefully) spread evenly across unvisited portion of surface           
        for (auto& testPt:testList){
            ScannedPoints.push_back(ScanPoint(testPt));
            //move sensor meshes to centre on testPt
            v = z_axis.cross(ScanPoint::pointCloud->normals_[testPt]);
            c = z_axis.dot(ScanPoint::pointCloud->normals_[testPt]);

            if (std::fabs(c+1)>=0.0001) { // normal pointing exactly opposite breaks following formula
                T.block<3,3>(0,0) = Eigen::Matrix3d::Identity() + skew(v) + skew(v)*skew(v) * (1/(1+c));
            }
            else { // rotate 180 in y 
                T.block<3,3>(0,0) = (Eigen::Matrix3d()<<-1.,0.,0.,0.,1.,0.,0.,0.,-1.).finished();
            }
            T.block<3,1>(0,3) = ScanPoint::pointCloud->points_[testPt];
            mesh_cone->Transform(T);
            mesh_cyl->Transform(T);  
            poly_cyl.Transform(T);

            //Searches only a sphere around centre pt - multiply fov by a little less than root 2 to get rounded corners
            pcd_tree.SearchRadius(ScanPoint::pointCloud->points_[testPt],(fov/2)*1.3,neighbours,neighbourDist);
            std::vector<int> windingNums;
            windingNums.reserve(neighbours.size());
            // find points that are within the focus region of the sensor 
            for (auto& i:neighbours){
                if (poly_cyl.winding_number(ScanPoint::pointCloud->points_[i]) == 1) {
                    windingNums.push_back(i);                
                }
            }
             
            for (auto& i :windingNums) {
                //start pt is just off surface, end pt is at sensor
                Eigen::Vector3d startPoint = ScanPoint::pointCloud->points_[i]+dof*ScanPoint::pointCloud->normals_[testPt]; //slight error in angle but makes sure first pt isnt on surface    
                Eigen::Vector3d endPoint = ScanPoint::pointCloud->points_[testPt]+(dof/2.0+standoff)*ScanPoint::pointCloud->normals_[testPt];
                bool collision = false;
                // draw a ray through octree to check if the sensor view is obstructed
                tree->computeRayKeys(octomap::point3d(startPoint[0],startPoint[1],startPoint[2]),octomap::point3d(endPoint[0],endPoint[1],endPoint[2]),rays);
                for (auto& ray:rays) {
                    octomap::OcTreeNode* node = tree->search(ray);
                    if(node!=0){
                        if(tree->isNodeOccupied(node)==1){
                            // std::cout<<"Collision"<<std::endl;
                            collision = true; 
                            break;
                        }
                    }
                }
                if (!collision) {
                    ScannedPoints.back().addViewPoint(i);
                }
            }       

            //put sensor mesh back
            T.block<3,3>(0,0).transposeInPlace();
            T.block<3,1>(0,3) = -1*T.block<3,3>(0,0)*T.block<3,1>(0,3);
            mesh_cone->Transform(T);
            mesh_cyl->Transform(T);
            poly_cyl.Transform(T);

            //check the validity of the surface angle wrt sensor and calculate ratio of good pts to bad
            ScannedPoints.back().getValidPoints();
            ScannedPoints.back().getRatio();
        }
        
        //Gone through a test list, now decide whether to use scanned points
        std::sort(ScannedPoints.begin(),ScannedPoints.end(),std::greater<ScanPoint>());
       
        for (std::vector<ScanPoint>::iterator sp = ScannedPoints.begin();sp!=ScannedPoints.end();++sp){
            
            //Check there are valid points in scanned point - can also use ratio, works better for dense pointclouds
            if (!(sp->used) && (sp->validPoints.size()>0)){ // && (sp->ratio>0.1)
                for (std::vector<int>::iterator unvisitedPts = sp->unvisitedPoints.begin();unvisitedPts!=sp->unvisitedPoints.end();){
                    std::vector<int>::iterator uvPt = std::find(unvisited.begin(), unvisited.end(), *unvisitedPts);
                    if (uvPt != unvisited.end()){
                        //remove all valid points from unvisited
                        unvisited.erase(uvPt);
                        sp->used = true;
                    }
                    else ++unvisitedPts;
                }
                if (sp->used) {
                    //Add colour to vis
                    ScanPoint::pointCloud->colors_[sp->centrePointIdx] = Eigen::Vector3d(0,0,1);
                    for (auto& validPt:sp->validPoints){
                        ScanPoint::pointCloud->colors_[validPt] += Eigen::Vector3d(-1,0.25,0);
                    }
                }
            }
        }
        // start a new test list - evenly spread through unvisited points
        // not a great strategy as doesn't guarentee test points are separate but using small batches (~20) reduces issue
        testList.clear();
        copynth(unvisited.begin(),unvisited.end(),std::back_inserter(testList),unvisited.size()/20);
        count ++;
        std::cout<<"Count: "<<count<<"\n";
        std::cout<<"Unvisited: "<<unvisited.size()<<"\n";
    }
    
    finished = true;
    render.join();
    
    //Finished finding scan points, now show them as arrows;

    std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> sensorPoints;
    
    for (std::vector<ScanPoint>::iterator sp = ScannedPoints.begin();sp!=ScannedPoints.end();++sp){
            if (sp->used) {
                v = z_axis.cross(sp->sensorNormal);
                c = z_axis.dot(sp->sensorNormal);

                if (std::fabs(c+1)>=0.0001) { // normal pointing exactly opposite breaks this formula
                    T.block<3,3>(0,0) = Eigen::Matrix3d::Identity() + skew(v) + skew(v)*skew(v) * (1/(1+c));
                }
                else{ // rotate 180 in y 
                    T.block<3,3>(0,0) = (Eigen::Matrix3d()<<-1.,0.,0.,0.,1.,0.,0.,0.,-1.).finished();
                }

                T.block<3,1>(0,3) = sp->sensorPosition;
                sensorPoints.push_back(open3d::geometry::TriangleMesh::CreateArrow(0.005,0.0075,0.05,0.025,20,4,1));
                sensorPoints.back()->Transform(T);
                sensorPoints.back()->PaintUniformColor((Eigen::Vector3d::Random(3,1).array()+1)/2);

            }
    }
    std::cout<<"Scan Points: "<<sensorPoints.size()<<std::endl;
    //open new vis that stays open
    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow();
    vis.AddGeometry(ScanPoint::pointCloud);
    for (auto& mesh : sensorPoints){
        vis.AddGeometry(mesh);
    }
   
    while (true) {
        if (!vis.PollEvents()){
            break;
            }
        vis.UpdateRender();     
    }
  
}

