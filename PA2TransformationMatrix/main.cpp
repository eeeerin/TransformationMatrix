//设T1为世界坐标系到一号自身坐标系的欧氏变换矩阵，T2为世界坐标系到二号自身坐标系的欧氏变换矩阵；p为该点在世界坐标系中的坐标，p2为该点在二号自身坐标系中的坐标。则：
//p=T1^-1 p1 （为了书写方便，实为齐次坐标变换）
//p2=T2 p
//
//        故欲求p2只需求得T1 T2
//        而已知旋转四元数和平移矩阵求欧氏变换矩阵只需将四元数转换成旋转向量，然后由旋转向量和平移矩阵求得欧氏变换矩阵即可

#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main( int argc, char** argv ) {

//    normalize
//    magnitude = sqrt(w2 + x2 + y2 + z2)
//    w = w / magnitude
//    x = x / magnitude
//    y = y / magnitude
//    z = z / magnitude

    //定义两个四元数
    Eigen::Quaterniond q1( 0.55,0.3,0.2,0.2 );
    Eigen::Quaterniond q2( -0.1,0.3,-0.7,0.2 );

    q1=q1.normalized();
    q2=q2.normalized();

    //将四元数转换成旋转矩阵
    Eigen::AngleAxisd rotation_vector1 = Eigen::AngleAxisd ( q1 );
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    T1.rotate( rotation_vector1 );
    T1.pretranslate( Eigen::Vector3d ( 0.7,1.1,0.2 ) );
    cout << "Transform matrix1 = \n " << T1.matrix() <<endl;

    //同上求出欧式变换矩阵T2
    Eigen::AngleAxisd rotation_vector2 = Eigen::AngleAxisd ( q2 );
    Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
    T2.rotate( rotation_vector2 );
    T2.pretranslate( Eigen::Vector3d ( -0.1,0.4,0.8 ) );
    cout << "Transform matrix2 = \n " << T2.matrix() <<endl;

    //计算该点在一号自身坐标系的坐标
    Eigen::Vector4d p1 ( 0.5,-0.1,0.2,1 );
    //计算p
    Eigen::Vector4d p = T1.inverse() * p1;
    cout << "p = \n " << p.transpose() << endl;

    Eigen::Vector4d p2 = T2 * p;
    cout << " p2 = \n " << p2.transpose() << endl;

    return 0;


}
