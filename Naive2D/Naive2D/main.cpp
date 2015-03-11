//
//  main.cpp
//  Naive2D
//
//  Created by Yinan Zhang on 3/11/15.
//  Copyright (c) 2015 Yinan Zhang. All rights reserved.
//

#include <iostream>
#include "geometry.h"
#include "polygon.h"
#include "render.h"

using namespace N2D;

void display()
{
    render::clean_screen();
    
    sphere sphere1(v2(200, 200), 100, SPHEREMETRIC::L1);
    Line_segment line(v2(200,200), v2(400, 400));
    v2 points[] = {v2(100, 100), v2(100, 200), v2(200, 150)};
    Polygon poly( points, 3 );
    
    render::sphere(sphere1, N2D::render::Color( 255, 0,0, 100 ));
    render::line_seg(line, N2D::render::Color( 0, 0, 255, 100 ));
    render::polygon(poly, N2D::render::Color( 0, 255, 0, 100 ));
    
    render::flush();
}

int main(int argc, const char * argv[])
{
    N2D::render::create_window(500, 500, "Rendering Test", v2(200, 200));
    N2D::render::set_display_func(display);
    N2D::render::main_loop();
    return 0;
}
