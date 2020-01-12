//
// Created by ricojia on 1/10/20.
//

#include "rigid2d.hpp"
#include <iosfwd>
#include <iostream>

using std::cout;
using std::cin;
using std::endl;

std::istream & test1(std::istream& is){
    rigid2d::Transform2D Tab, Tbc;
    rigid2d::Vector2D v_c;
    is>>Tab;   //theta, x, y
    is>>Tbc;   // theta, x, y
    is>>v_c;
    is.ignore(INT_MAX);
    rigid2d::Transform2D Tac = Tab*Tbc;
    rigid2d::Vector2D v_a = Tac(v_c);
    cout<<v_a;
    return is;
}


int main(){
//    rigid2d::Transform2D Tab, Tbc;
//    cout<<"Welcome! Now enter theta, x, y for Tab"<<endl;
//    cin>>Tab;
//    cout<<"Now enter theta, x, y for Tbc"<<endl;
//    cin>>Tbc;
//    rigid2d::Transform2D Tac = Tab*Tbc;
//    rigid2d::Transform2D Tca = Tac.inv();
//    rigid2d::Transform2D Tba = Tab.inv();
//    rigid2d::Transform2D Tcb = Tbc.inv();
//    cout<<"Tab is: "<<endl<<Tab;
//    cout<<"Tbc is: "<<endl<<Tbc;
//    cout<<"Tac is: "<<endl<<Tac;
//    cout<<"Tca is: "<<endl<<Tca;
//
//    char frame;
//    rigid2d::Vector2D v;
//    rigid2d::Twist2D t;
//    cout<<"OK, now enter a 2D vector, x, y"<<endl;
//    cin>>v;
//    cout<<"OK, now enter a 2D twist in the same frame, theta, x, y"<<endl;
//    cin>>t;
//    cout<<"Now please enter the frame the above entries are represented in"<<endl;
//    cin>>frame;
//    size_t index = frame - 'a';
//    rigid2d::Transform2D T_arr[] = {Tba, Tcb, Tac, Tba, Tcb};
//    char names[] = {'a', 'b', 'c','a','b'};
//    cout<<"Now the vectors are: "<<endl<<"V"<<names[index]<<v<<endl
//        <<", V"<<names[index+1]<<T_arr[index](v)<<endl
//        <<", V"<<names[index+2]<<T_arr[index+1](T_arr[index](v))<<endl;
//
//    cout<<"Now the twists are: "<<endl<<"T"<<names[index]<<t<<endl
//        <<", T"<<names[index+1]<<T_arr[index](t)<<endl
//        <<", T"<<names[index+2]<<T_arr[index+1](T_arr[index](t))<<endl;

    test1(cin);
    return 0;
}

