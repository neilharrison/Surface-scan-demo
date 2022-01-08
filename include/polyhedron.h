#include <iostream>
#include <Eigen/Eigen>

// https://github.com/mdickinson/polyhedron
// code from above re-written in C++ using Eigen
// explanation and algorithm found in git repo

int sign(const double &x){
   /*
    Return 1 if x is positive, -1 if it's negative, and 0 if it's zero.
   */
    if (x > 0){
        return 1;
    }
    else if (x < 0) {
        return -1;
    }
    else {
        return 0; 
    }
} 

int vertex_sign(Eigen::Vector3d &P, Eigen::Vector3d &O){
    /*
    Sign of the vertex P with respect to O, as defined above.
    */
    int result;
    if (result = sign(P[0] - O[0])) ;

    else if (result = sign(P[1] - O[1])) ;

    else if (result = sign(P[2] - O[2])!=0) ;

    else {
        std::cout<<"vertex coincides with origin"<<std::endl;
        //Should raise error here!@
    }
    return result;
}

int edge_sign(Eigen::Vector3d &P,Eigen::Vector3d &Q,Eigen::Vector3d &O){
    /*
    Sign of the edge PQ with respect to O, as defined above.
    */
    int result;
    if (result = sign((P[1] - O[1]) * (Q[0] - O[0]) - (P[0] - O[0]) * (Q[1] - O[1])));

    else if (result = sign((P[2] - O[2]) * (Q[0] - O[0]) - (P[0] - O[0]) * (Q[2] - O[2])));

    else if (result = sign((P[2] - O[2]) * (Q[1] - O[1]) - (P[1] - O[1]) * (Q[2] - O[2])));
    else{
        //should Throw proper error here
        std::cout<<"vertices collinear with origin"<<std::endl;
    }
    return result;
}


int triangle_sign(Eigen::Vector3d &P,Eigen::Vector3d &Q,Eigen::Vector3d &R,Eigen::Vector3d &O) {
   /*
    Sign of the triangle PQR with respect to O, as defined above.
    */

    double m1_0 = P[0] - O[0];
    double m1_1 = P[1] - O[1];
    double m2_0 = Q[0] - O[0];
    double m2_1 = Q[1] - O[1];
    double m3_0 = R[0] - O[0];
    double m3_1 = R[1] - O[1];

    int result = sign(
        (m1_0 * m2_1 - m1_1 * m2_0) * (R[2] - O[2]) +
        (m2_0 * m3_1 - m2_1 * m3_0) * (P[2] - O[2]) +
        (m3_0 * m1_1 - m3_1 * m1_0) * (Q[2] - O[2]));
    if (result == 0) {
         //Throw error here
        std::cout<<"vertices coplanar with origin"<<std::endl;       
    }
    return result;
}

int triangle_chain(Eigen::Vector3d &v1,Eigen::Vector3d &v2,Eigen::Vector3d &v3,Eigen::Vector3d &origin){
    /*
    Return the contribution of this triangle to the winding number.

    Raise ValueError if the face contains the origin.
    */
    int v1sign = vertex_sign(v1, origin);
    int v2sign = vertex_sign(v2, origin);
    int v3sign = vertex_sign(v3, origin);

    int face_boundary = 0;

    if (v1sign != v2sign){
        face_boundary += edge_sign(v1, v2, origin);
    }
    if (v2sign != v3sign){
        face_boundary += edge_sign(v2, v3, origin);
    }
    if (v3sign != v1sign) {
        face_boundary += edge_sign(v3, v1, origin);
    }
    if (face_boundary == 0){
        return 0;
    }
        
    return triangle_sign(v1, v2, v3, origin);
}

class Polyhedron{
    public:
    std::vector<Eigen::Vector3i> triangles;
    std::vector<Eigen::Vector3d> vertex_positions;
    std::vector<std::vector<Eigen::Vector3d>> triangle_pos;

    Polyhedron(){}
    Polyhedron(std::vector<Eigen::Vector3i> &_triangles,std::vector<Eigen::Vector3d> &_vertex_positions){
       /*
        Initialize from list of triangles and vertex positions.
        */
        // TO DO Validate: check the combinatorial data. 
        //- should check that the mesh is intact but as all meshes are generated from open3d they shouldnt be invalid
 
        vertex_positions = _vertex_positions;
        //Indices making up each triangle, counterclockwise
        // around the outside of the face.
        triangles = _triangles;
        triangle_pos = triangle_positions();
    }

    std::vector<std::vector<Eigen::Vector3d>>  triangle_positions(){
        /*
        Triples of vertex positions.
        */
        
        for (std::vector<Eigen::Vector3i>::iterator triangle = triangles.begin();triangle!=triangles.end();++triangle) {
            std::vector<Eigen::Vector3d> vx = {Eigen::Vector3d(vertex_positions[triangle[0][0]]),Eigen::Vector3d(vertex_positions[triangle[0][1]]),Eigen::Vector3d(vertex_positions[triangle[0][2]])};
            triangle_pos.push_back(vx);
        }
        return triangle_pos;
    }

    double volume() {
        /*
        Return the volume of this polyhedron.
        */
        double acc = 0;
        for (int i = 0;i<triangle_pos.size();i++){
            Eigen::Vector3d p1 = triangle_pos[i][0];
            Eigen::Vector3d p2 = triangle_pos[i][1];
            Eigen::Vector3d p3 = triangle_pos[i][2];
            double det = ((p2[1] - p3[1]) * (p1[0] - p3[0]) -
                   (p2[0] - p3[0]) * (p1[1] - p3[1]));
            // Three times the average height.
            double height = p1[2] + p2[2] + p3[2];
            acc += det * height;
        }
        return acc / 6.0;
    }

    int winding_number(Eigen::Vector3d &point){
        /*Determine the winding number of *self* around the given point.
        */  
        int sum = 0;
        for (auto& triangle : triangle_pos) {
            sum += triangle_chain(triangle[0], triangle[1], triangle[2], point);
        }
        return sum/2;
    }
    //Stolen from open3d geometry
    void Transform(const Eigen::Matrix4d& transformation) {
    for (auto& triple : triangle_pos) {
        for (auto& point : triple) {
            Eigen::Vector4d new_point =
                    transformation *
                    Eigen::Vector4d(point(0), point(1), point(2), 1.0);
            point = new_point.head<3>() / new_point(3);
        } 
    }
}


};