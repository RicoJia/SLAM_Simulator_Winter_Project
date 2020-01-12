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
    test1(cin);
    return 0;
}

//
// Created by ricojia on 1/11/20.
//

