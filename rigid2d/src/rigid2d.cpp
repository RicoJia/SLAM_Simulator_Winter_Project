#include "rigid2d/rigid2d.hpp"

using std::cout;
using std::cin;
using std::endl;


std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Vector2D & v){
    os<<"["<<v.x<<endl<<" "<<v.y<<"]"<<endl;
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Vector2D & v){
    is>>v.x;
    is>>v.y;
    return is;
}

std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Transform2D & tf){
    os<<"******************"<<endl<<tf.T<<endl<<"******************"<<endl;
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Transform2D & tf){
    double theta;
    rigid2d::Vector2D v;
    is>>theta;
    is>>v.x;
    is>>v.y;
    rigid2d::Transform2D new_t(v, theta);
    tf = new_t;
    return is;
}

rigid2d::Transform2D::Transform2D(){
    T = MatrixXd::Identity(3, 3);
}

rigid2d::Transform2D::Transform2D(const rigid2d::Vector2D & trans){
    T = MatrixXd::Identity(3, 3);
    T(0,2) = trans.x;
    T(1,2) = trans.y;
}

rigid2d::Transform2D::Transform2D(double radians){
    T = MatrixXd::Identity(3, 3);
    double c = cos(radians);
    double s = sin(radians);
    T(0,0) = c; T(0,1) = -1.0*s;
    T(1,0) = s; T(1,1) = c;
}

rigid2d::Transform2D::Transform2D(const rigid2d::Vector2D &trans, double radians) {
    T = MatrixXd::Identity(3, 3);
    double c = cos(radians);
    double s = sin(radians);
    T(0,0) = c; T(0,1) = -1.0*s; T(0,2) = trans.x;
    T(1,0) = s; T(1,1) = c; T(1,2) = trans.y;
}

rigid2d::Vector2D rigid2d::Transform2D::operator()(rigid2d::Vector2D v) const {
    Vector3d u(v.x, v.y, 1.0);
    Vector3d u_trans = T*u;
    return Vector2D(u_trans(0), u_trans(1));
}

rigid2d::Transform2D rigid2d::Transform2D::inv() const {
    auto T_inv = T.inverse();
    rigid2d::Vector2D p_inv(T_inv(0,2), T_inv(1,2));
    double theta_inv = -1*atan2(T(1,0), T(0,0));
    rigid2d::Transform2D trans_inv(p_inv, theta_inv);
    return trans_inv;
}

rigid2d::Transform2D & rigid2d::Transform2D::operator*=(const rigid2d::Transform2D & rhs){
    T = T*rhs.T;
    return *this;
}

rigid2d::Transform2D rigid2d::operator*(rigid2d::Transform2D lhs, const rigid2d::Transform2D &rhs) {
    return lhs*=rhs;
}


std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Twist2D & v){
    os<<"["<<v.theta<<endl<<" "<<v.x<<endl<<" "<<v.y<<"]"<<endl;
    return os;
}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Twist2D & v){
    is>>v.theta;
    is>>v.x;
    is>>v.y;
    return is;
}
rigid2d::Twist2D rigid2d::Transform2D::operator()(rigid2d::Twist2D v) const{
    auto adj = T;
    adj(0,2) = T(1,2);
    adj(1,2) = -1*T(0,2);
    Vector3d v_vec(v.x, v.y, v.theta);
    auto new_v = adj*v_vec;
    rigid2d::Twist2D ret_v(new_v(2), new_v(0), new_v(1));
    return ret_v;

}