//create rectangular based pyramid mesh using open3d

std::shared_ptr<open3d::geometry::TriangleMesh> CreateRectPyramid(
        double base1 /* = 1.0*/,
        double base2 /* = 1.0*/,
        double height /* = 1.0*/) {
    
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    if (base1 <= 0) {
        open3d::utility::LogError("[CreatePyramid] base1 <= 0");
    }
    if (base2 <= 0) {
        open3d::utility::LogError("[CreatePyramid] base2 <= 0");
    }
    if (height <= 0) {
        open3d::utility::LogError("[CreatePyramid] height <= 0");
    }
    //origin at central axis on base
    // Vertices.
    mesh->vertices_.resize(5);
    mesh->vertices_[0] = Eigen::Vector3d(-base1/2, -base2/2, 0.0);
    mesh->vertices_[1] = Eigen::Vector3d(-base1/2, base2/2, 0.0);
    mesh->vertices_[2] = Eigen::Vector3d(base1/2, -base2/2, 0.0);
    mesh->vertices_[3] = Eigen::Vector3d(base1/2, base2/2, 0.0);
    mesh->vertices_[4] = Eigen::Vector3d(0.0, 0.0, height);

    // Triangles.
    mesh->triangles_ = {{0, 1, 2}, {1, 3, 2}, {0, 4, 1}, {1, 4, 3},
                        {2, 3, 4}, {0, 2, 4}};

    return mesh;
}

Eigen::Matrix3d skew(Eigen::Vector3d vector){
    return (Eigen::Matrix3d()<<0,-vector[2],vector[1],
                            vector[2],  0, -vector[0],
                            -vector[1],vector[0],0).finished();
}

//copied from somewhere on stackoverflow
template<typename T, typename U>
void copynth( T begin , T end , U dest , int n )
{
    if (n == 0) n=1;
    int count = 0;
    std::copy_if( begin , end , dest ,
        [&count,n]( double x )
    {
        return count++ % n == 0;
    });
}
