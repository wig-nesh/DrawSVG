#include "software_renderer.h"

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <limits.h>
#include <stack>

#include "triangulation.h"

using namespace std;

namespace CMU462 {


// Implements SoftwareRenderer //

void SoftwareRendererImp::draw_svg( SVG& svg ) {

  // set top level transformation
  transformation = svg_2_screen;

  // draw all elements
  for ( size_t i = 0; i < svg.elements.size(); ++i ) {
    draw_element(svg.elements[i]);
  }

  // draw canvas outline
  Vector2D a = transform(Vector2D(    0    ,     0    )); a.x--; a.y--;
  Vector2D b = transform(Vector2D(svg.width,     0    )); b.x++; b.y--;
  Vector2D c = transform(Vector2D(    0    ,svg.height)); c.x--; c.y++;
  Vector2D d = transform(Vector2D(svg.width,svg.height)); d.x++; d.y++;

  rasterize_line(a.x, a.y, b.x, b.y, Color::Black);
  rasterize_line(a.x, a.y, c.x, c.y, Color::Black);
  rasterize_line(d.x, d.y, b.x, b.y, Color::Black);
  rasterize_line(d.x, d.y, c.x, c.y, Color::Black);

  // resolve and send to render target
  resolve();

}

void SoftwareRendererImp::set_sample_rate( size_t sample_rate ) {

  // Task 4: 
  // You may want to modify this for supersampling support
  this->sample_rate = sample_rate;

}

void SoftwareRendererImp::set_render_target( unsigned char* render_target,
                                             size_t width, size_t height ) {

  // Task 4: 
  // You may want to modify this for supersampling support
  this->render_target = render_target;
  this->target_w = width;
  this->target_h = height;

}

void SoftwareRendererImp::draw_element( SVGElement* element ) {

  // Task 5 (part 1):
  // Modify this to implement the transformation stack

  switch(element->type) {
    case POINT:
      draw_point(static_cast<Point&>(*element));
      break;
    case LINE:
      draw_line(static_cast<Line&>(*element));
      break;
    case POLYLINE:
      draw_polyline(static_cast<Polyline&>(*element));
      break;
    case RECT:
      draw_rect(static_cast<Rect&>(*element));
      break;
    case POLYGON:
      draw_polygon(static_cast<Polygon&>(*element));
      break;
    case ELLIPSE:
      draw_ellipse(static_cast<Ellipse&>(*element));
      break;
    case IMAGE:
      draw_image(static_cast<Image&>(*element));
      break;
    case GROUP:
      draw_group(static_cast<Group&>(*element));
      break;
    default:
      break;
  }

}


// Primitive Drawing //

void SoftwareRendererImp::draw_point( Point& point ) {

  Vector2D p = transform(point.position);
  rasterize_point( p.x, p.y, point.style.fillColor );

}

void SoftwareRendererImp::draw_line( Line& line ) { 

  Vector2D p0 = transform(line.from);
  Vector2D p1 = transform(line.to);
  rasterize_line( p0.x, p0.y, p1.x, p1.y, line.style.strokeColor );

}

void SoftwareRendererImp::draw_polyline( Polyline& polyline ) {

  Color c = polyline.style.strokeColor;

  if( c.a != 0 ) {
    int nPoints = polyline.points.size();
    for( int i = 0; i < nPoints - 1; i++ ) {
      Vector2D p0 = transform(polyline.points[(i+0) % nPoints]);
      Vector2D p1 = transform(polyline.points[(i+1) % nPoints]);
      rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    }
  }
}

void SoftwareRendererImp::draw_rect( Rect& rect ) {

  Color c;
  
  // draw as two triangles
  float x = rect.position.x;
  float y = rect.position.y;
  float w = rect.dimension.x;
  float h = rect.dimension.y;

  Vector2D p0 = transform(Vector2D(   x   ,   y   ));
  Vector2D p1 = transform(Vector2D( x + w ,   y   ));
  Vector2D p2 = transform(Vector2D(   x   , y + h ));
  Vector2D p3 = transform(Vector2D( x + w , y + h ));
  
  // draw fill
  c = rect.style.fillColor;
  if (c.a != 0 ) {
    rasterize_triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c );
    rasterize_triangle( p2.x, p2.y, p1.x, p1.y, p3.x, p3.y, c );
  }

  // draw outline
  c = rect.style.strokeColor;
  if( c.a != 0 ) {
    rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    rasterize_line( p1.x, p1.y, p3.x, p3.y, c );
    rasterize_line( p3.x, p3.y, p2.x, p2.y, c );
    rasterize_line( p2.x, p2.y, p0.x, p0.y, c );
  }

}

void SoftwareRendererImp::draw_polygon( Polygon& polygon ) {

  Color c;

  // draw fill
  c = polygon.style.fillColor;
  if( c.a != 0 ) {

    // triangulate
    vector<Vector2D> triangles;
    triangulate( polygon, triangles );

    // draw as triangles
    for (size_t i = 0; i < triangles.size(); i += 3) {
      Vector2D p0 = transform(triangles[i + 0]);
      Vector2D p1 = transform(triangles[i + 1]);
      Vector2D p2 = transform(triangles[i + 2]);
      rasterize_triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c );
    }
  }

  // draw outline
  c = polygon.style.strokeColor;
  if( c.a != 0 ) {
    int nPoints = polygon.points.size();
    for( int i = 0; i < nPoints; i++ ) {
      Vector2D p0 = transform(polygon.points[(i+0) % nPoints]);
      Vector2D p1 = transform(polygon.points[(i+1) % nPoints]);
      rasterize_line( p0.x, p0.y, p1.x, p1.y, c );
    }
  }
}

void SoftwareRendererImp::draw_ellipse( Ellipse& ellipse ) {

  // Extra credit 

}

void SoftwareRendererImp::draw_image( Image& image ) {

  Vector2D p0 = transform(image.position);
  Vector2D p1 = transform(image.position + image.dimension);

  rasterize_image( p0.x, p0.y, p1.x, p1.y, image.tex );
}

void SoftwareRendererImp::draw_group( Group& group ) {

  for ( size_t i = 0; i < group.elements.size(); ++i ) {
    draw_element(group.elements[i]);
  }

}

// Rasterization //

// The input arguments in the rasterization functions 
// below are all defined in screen space coordinates

void SoftwareRendererImp::rasterize_point( float x, float y, Color color ) {

  // fill in the nearest pixel
  int sx = (int) floor(x);
  int sy = (int) floor(y);

  // check bounds
  if ( sx < 0 || sx >= target_w ) return;
  if ( sy < 0 || sy >= target_h ) return;

  // fill sample - NOT doing alpha blending!
  render_target[4 * (sx + sy * target_w)    ] = (uint8_t) (color.r * 255);
  render_target[4 * (sx + sy * target_w) + 1] = (uint8_t) (color.g * 255);
  render_target[4 * (sx + sy * target_w) + 2] = (uint8_t) (color.b * 255);
  render_target[4 * (sx + sy * target_w) + 3] = (uint8_t) (color.a * 255);

}

void SoftwareRendererImp::rasterize_line( float x0, float y0,
                                          float x1, float y1,
                                          Color color) {

  // Task 2: 
  // Implement line rasterization

  float m = (y1-y0) / (x1-x0); // slope
  float e = 0;             // error

  if (m <= 1 && m >= -1) {
    if (x1 < x0) { // if x1 < x0, we need to flip the vector
      swap(x0, x1); 
      swap(y0, y1);
    }
    float y = y0;
    for (float x = x0; x <= x1; x+=1.0) { // traverse the length of vector
      rasterize_point(x, y, color);
      if (m >= 0 && m <= 1) { // positive slope
        if (e + m < 0.5) e += m;
        else {
          y += 1.0;
          e += m - 1;
        }
      }
      else if (m < 0 && m >= -1){ // negative slope
        if (e + m > -0.5) e += m;
        else {
          y -= 1.0;
          e += m + 1;
        }
      }
    }
  }
  else {
    if (y1 < y0) { // if y1 < y0, we need to flip the vector
      swap(x0, x1); 
      swap(y0, y1);
    }
    float x = x0;
    for (float y = y0; y <= y1; y+=1.0) { // traverse the length of vector
      rasterize_point(x, y, color);
      if (m > 1 && m < numeric_limits<float>::max()) { // positive slope
        if (e + 1/m < 0.5) e += 1/m;
        else {
          x += 1.0;
          e += 1/m - 1;
        }
      }
      else if (m < -1 && m > -numeric_limits<float>::max()){ // negative slope
        if (e + 1/m > -0.5) e += 1/m;
        else {
          x -= 1.0;
          e += 1/m + 1;
        }
      }
    }
  }

}

bool linesIntersect(Vector2D p1, Vector2D p2, Vector2D q1, Vector2D q2) {
    float s1_x = p2.x - p1.x;
    float s1_y = p2.y - p1.y;
    float s2_x = q2.x - q1.x;
    float s2_y = q2.y - q1.y;

    float s = (-s1_y * (p1.x - q1.x) + s1_x * (p1.y - q1.y)) / (-s2_x * s1_y + s1_x * s2_y);
    float t = ( s2_x * (p1.y - q1.y) - s2_y * (p1.x - q1.x)) / (-s2_x * s1_y + s1_x * s2_y);

    return (s >= 0 && s <= 1 && t >= 0 && t <= 1);
}


void SoftwareRendererImp::rasterize_triangle( float x0, float y0, // A
                                              float x1, float y1, // B
                                              float x2, float y2, // C
                                              Color color ) {
  // Task 3: 
  // Implement triangle rasterization

  Vector2D A = Vector2D(x0,y0);
  Vector2D B = Vector2D(x1,y1);
  Vector2D C = Vector2D(x2,y2);

  Color boxCol;
  boxCol.r = 0;
  boxCol.g = 1;
  boxCol.b = 0;
  boxCol.a = 1;

  Color triangleCol;
  triangleCol.r = 0;
  triangleCol.g = 0;
  triangleCol.b = 1;
  triangleCol.a = 1;

  // triangle visualization
  rasterize_line(A.x, A.y, B.x, B.y, triangleCol);
  rasterize_line(B.x, B.y, C.x, C.y, triangleCol);
  rasterize_line(C.x, C.y, A.x, A.y, triangleCol);

  // narrow down search area to bounding box
  float top = min({A.y, B.y, C.y});
  float bottom = max({A.y, B.y, C.y});
  float left = min({A.x, B.x, C.x});
  float right = max({A.x, B.x, C.x});

  // edge vectors
  Vector2D AB = B - A;
  Vector2D BC = C - B;
  Vector2D CA = A - C;

  int max_depth = 3;
  stack<pair<vector<float>, int>> stack;
  stack.push({{top, bottom, left, right}, 0});

  while (!stack.empty()) {
    auto [c, depth] = stack.top();
    stack.pop();

    float cTop = c[0];
    float cBottom = c[1];
    float cLeft = c[2];
    float cRight = c[3];
    Vector2D P = Vector2D(cLeft, cTop);
    Vector2D Q = Vector2D(cRight, cTop);
    Vector2D R = Vector2D(cRight, cBottom);
    Vector2D S = Vector2D(cLeft, cBottom);

    // only continue if bounding box intersects triangle
    bool ignore_box = true;
    // first check if any vertices of the box intersects the triangle
    float arr[8] = {cLeft, cTop, cRight, cTop, cLeft, cBottom, cRight, cBottom};
    for (int it = 0; it < 8; it+=2) {
      Vector2D AP = Vector2D(arr[it]-x0, arr[it+1]-y0);
      Vector2D BP = Vector2D(arr[it]-x1, arr[it+1]-y1);
      Vector2D CP = Vector2D(arr[it]-x2, arr[it+1]-y2);
      float crossA = AB.x*AP.y - AB.y*AP.x;
      float crossB = BC.x*BP.y - BC.y*BP.x;
      float crossC = CA.x*CP.y - CA.y*CP.x;
      if (crossA<0 && crossB<0 && crossC<0 || crossA>=0 && crossB>=0 && crossC>=0) {
        ignore_box = false;
        break;
      }
    }
    // if it passes the previous check, check edges
    if (ignore_box) {
      Vector2D box_edge_arr[8] = {P, Q, Q, R, R, S, S, P};
      Vector2D tri_edge_arr[6] = {A, B, B, C, C, A};
      for (int it = 0; it < 8; it+=2) {
        Vector2D rectQ = box_edge_arr[it];
        Vector2D rectR = box_edge_arr[it+1];
        for (int it2 = 0; it2 < 6; it2+=2) {
          Vector2D triA = tri_edge_arr[it2];
          Vector2D triB = tri_edge_arr[it2+1];
          ignore_box = !linesIntersect(triA, triB, rectQ, rectR);
          if (!ignore_box) break;
        }
        if (!ignore_box) break;
      }
    }

    if (!ignore_box) {
      if (depth == max_depth) {
        for (float i = floor(cLeft)+0.5; i <= floor(cRight)+0.5; i+=1.0) {
          for (float j = floor(cTop)+0.5; j <= floor(cBottom)+0.5; j+=1.0) {
            Vector2D AP = Vector2D(i-x0, j-y0);
            Vector2D BP = Vector2D(i-x1, j-y1);
            Vector2D CP = Vector2D(i-x2, j-y2);
            // 2d cross products
            float crossA = AB.x*AP.y - AB.y*AP.x;
            float crossB = BC.x*BP.y - BC.y*BP.x;
            float crossC = CA.x*CP.y - CA.y*CP.x;
            // signs are same
            if (crossA<0 && crossB<0 && crossC<0 || crossA>=0 && crossB>=0 && crossC>=0) {
              rasterize_point(i, j, color);
            }
          }
        }
        // box visualizations
        rasterize_line(cLeft, cTop, cRight, cTop, boxCol);
        rasterize_line(cLeft, cTop, cLeft, cBottom, boxCol);
        rasterize_line(cRight, cBottom, cLeft, cBottom, boxCol);
        rasterize_line(cRight, cBottom, cRight, cTop, boxCol);
      }
      else {
        float midVertical = cTop + (cBottom - cTop) / 2;
        float midHorizontal = cLeft + (cRight - cLeft) / 2;
        stack.push({{cTop, midVertical, cLeft, midHorizontal}, depth + 1});
        stack.push({{cTop, midVertical, midHorizontal, cRight}, depth + 1});
        stack.push({{midVertical, cBottom, cLeft, midHorizontal}, depth + 1});
        stack.push({{midVertical, cBottom, midHorizontal, cRight}, depth + 1});
      }
    }
    else {
      // more box visualizations
      rasterize_line(cLeft, cTop, cRight, cTop, boxCol);
      rasterize_line(cLeft, cTop, cLeft, cBottom, boxCol);
      rasterize_line(cRight, cBottom, cLeft, cBottom, boxCol);
      rasterize_line(cRight, cBottom, cRight, cTop, boxCol);
    }
  }
}

void SoftwareRendererImp::rasterize_image( float x0, float y0,
                                           float x1, float y1,
                                           Texture& tex ) {
  // Task 6: 
  // Implement image rasterization

}

// resolve samples to render target
void SoftwareRendererImp::resolve( void ) {

  // Task 4: 
  // Implement supersampling
  // You may also need to modify other functions marked with "Task 4".
  return;

}


} // namespace CMU462
