#pragma once

#include <GL/gl.h>
#include <GL/glut.h>

namespace pwn_viewer {

  /**
   * Draw a box that is centered in the current coordinate frame
   * @param l length of the box (x dimension)
   * @param w width of the box (y dimension)
   * @param h height of the box (z dimension)
   */
  void drawBox(GLfloat l, GLfloat w, GLfloat h);

  /**
   * Draw a plane in x-y dimension with a height of zero
   * @param l length in x
   * @param w width in y
   */
  void drawPlane(GLfloat l, GLfloat w);

  /**
   * Draw a sphere whose center is in the origin of the current coordinate frame
   * @param radius the radius of the sphere
   */
  void drawSphere(GLfloat radius);

  /**
   * Draw a ellipsoid whose center is in the origin of the current coordinate frame
   * @param r1 radius along x axis
   * @param r2 radius along y axis
   * @param r3 radius along z axis
   */
  void drawEllipsoid(GLfloat r1, GLfloat r2, GLfloat r3);

  /**
   * Draw a cone
   */
  void drawCone(GLfloat radius, GLfloat height);

  /**
   * Draw a disk
   */
  void drawDisk(GLfloat radius);

  /**
   * Draw a (closed) cylinder
   * @param radius the radius of the cylinder
   * @param height the height of the cylinder
   */
  void drawCylinder(GLfloat radius, GLfloat height);

  /**
   * Draw a pyramid
   */
  void drawPyramid(GLfloat length, GLfloat height);

  /**
   * Draw a range ring
   * @param range the range (radius) of the partial ring
   * @param fov Field Of View of the range sensor
   * @param range_width specify how thick the ring should be drawn
   */
  void drawRangeRing(GLfloat range, GLfloat fov, GLfloat range_width = 0.05);

  /**
   * Draw a slice of a cylinder (approximated with slices_per_circle triangles for the complete circle)
   * @param radius the radius of the cylinder
   * @param height the height of the cylinder
   * @param fov the "fov" of the slice (om degree)
   * @param slices_per_circle the number of triangle used to approximate the fulle circle
   */
  void drawSlice(GLfloat radius, GLfloat height, GLfloat fov, int slices_per_circle = 32);

  /**
   * Draws a box used to represent a 6d pose
   */
  void drawPoseBox();

  /**
   * Draw a 2D arrow along the x axis with the given len
   */
  void drawArrow2D(float len, float head_width, float head_len);

  /**
   * Draw a point in the origin, having a size of pointSize
   */
  void drawPoint(float  pointSize);

}
