// Copyright (c) 2015,2016 GeometryFactory
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
// You can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL$
// $Id$
//
//
// Author(s)     : Laurent Rineau

#ifndef CGAL_MESH_3_POLYLINES_TO_PROTECT_H
#define CGAL_MESH_3_POLYLINES_TO_PROTECT_H

#include <vector>
#include <map>
#include <utility> // std::swap
#include <CGAL/tuple.h>
#include <CGAL/Image_3.h>
#include <CGAL/boost/graph/split_graph_into_polylines.h>
#include <CGAL/internal/Mesh_3/Graph_manipulations.h>
#include <boost/graph/adjacency_list.hpp>
#include <CGAL/Labeled_image_mesh_domain_3.h> // for
                                              // CGAL::Null_subdomain_index
#include <boost/utility.hpp> // for boost::prior
#include <boost/foreach.hpp>

#include <CGAL/Search_traits_3.h>
#include <CGAL/Orthogonal_incremental_neighbor_search.h>

namespace CGAL {

namespace Mesh_3{
template<typename P, typename G>
struct Polyline_visitor
{
  std::vector<std::vector<P> >& polylines;
  G& graph;
  Polyline_visitor(typename std::vector<std::vector<P> >& lines, G& p_graph)
    : polylines(lines), graph(p_graph)
  {}

  void start_new_polyline()
  {
    std::vector<P> V;
    polylines.push_back(V);
  }

  void add_node(typename boost::graph_traits<G>::vertex_descriptor vd)
  {
    std::vector<P>& polyline = polylines.back();
    polyline.push_back(graph[vd]);
  }

  void end_polyline()
  {
    // ignore degenerated polylines
    if(polylines.back().size() < 2)
      polylines.resize(polylines.size() - 1);
  }
};
}//namespace Mesh_3

// this function is overloaded for when `PolylineInputIterator` is `int`.
template <typename K,
          typename Graph,
          typename PolylineInputIterator>
void snap_graph_vertices(Graph& graph,
                         const double vx, const double vy, const double vz,
                         PolylineInputIterator existing_polylines_begin,
                         PolylineInputIterator existing_polylines_end,
                         K)
{
  const double dist_bound = (std::min)(vx,
                                       (std::min)(vy, vz)) / 256;
  const double sq_dist_bound = dist_bound * dist_bound;

  typedef CGAL::Search_traits_3<K> Tree_traits;
  typedef CGAL::Orthogonal_incremental_neighbor_search<Tree_traits> NN_search;
  typedef typename NN_search::Tree Tree;

  Tree tree;
  // insert the extremities of the polylines in the kd-tree
  for(PolylineInputIterator poly_it = existing_polylines_begin;
      poly_it != existing_polylines_end; ++poly_it)
  {
    if(poly_it->begin() != poly_it->end()) {
      tree.insert(*poly_it->begin());
      if(boost::next(poly_it->begin()) != poly_it->end()) {
        tree.insert(*boost::prior(poly_it->end()));
      }
    }
  }
  if(tree.size() == 0) return;

  BOOST_FOREACH(typename boost::graph_traits<Graph>::vertex_descriptor v,
                vertices(graph))
  {
    const typename K::Point_3 p = graph[v];
    NN_search nn(tree, p);
    CGAL_assertion(nn.begin() != nn.end());
    if(squared_distance(nn.begin()->first, p) < sq_dist_bound) {
      graph[v] = nn.begin()->first;
    }
  }
}

template <typename K,
          typename Graph>
void snap_graph_vertices(Graph&, double, double, double, int, int, K)
{}

template <typename P,
          typename Image_word_type,
          typename Null_subdomain_index,
          typename PolylineInputIterator>
void
polylines_to_protect(const CGAL::Image_3& cgal_image,
                     const double vx, const double vy, const double vz,
                     std::vector<std::vector<P> >& polylines,
                     Image_word_type*,
                     Null_subdomain_index null,
                     PolylineInputIterator existing_polylines_begin,
                     PolylineInputIterator existing_polylines_end)
{
  typedef typename Kernel_traits<P>::Kernel K;
  typedef P Point_3;
  typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, Point_3> Graph;
  typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
  // typedef typename boost::graph_traits<Graph>::edge_iterator edge_iterator;

  const int xdim = static_cast<int>(cgal_image.xdim());
  const int ydim = static_cast<int>(cgal_image.ydim());
  const int zdim = static_cast<int>(cgal_image.zdim());

  const int image_dims[3] = { xdim, ydim, zdim };

  Graph graph;
  internal::Mesh_3::Graph_manipulations<Graph, Point_3> g_manip(graph);

  std::size_t
    case4 = 0, // 4 colors
    case211 = 0, case121 = 0, // 3 colors
    case31 = 0, case22 = 0, // 2 colors
    case1 = 0; // 1 color

  for(int axis = 0; axis < 3; ++axis)
  {
    for(int i = 0; i < xdim; i+= (axis == 0 ? (xdim-1) : 1 ) )
      for(int j = 0; j < ydim; j+= (axis == 1 ? (ydim-1) : 1 ) )
        for(int k = 0; k < zdim; k+= (axis == 2 ? (zdim-1) : 1 ) )
        {

          using CGAL::cpp11::array;
          using CGAL::cpp11::tuple;
          using CGAL::cpp11::get;

          typedef array<int, 3> Pixel;

          Pixel pix00 = {{i  , j  , k  }},
            pix10 = pix00, pix01 = pix00, pix11 = pix00;

          const int axis_xx = (axis + 1) % 3;
          const int axis_yy = (axis + 2) % 3;

          ++pix10[axis_xx];
          ++pix11[axis_xx]; ++pix11[axis_yy];
          ++pix01[axis_yy];
          if(pix11[0] >= xdim || pix11[1] >= ydim || pix11[2] >= zdim) {
            // we have gone too far
            continue;
          }
          typedef tuple<Pixel, Point_3, Image_word_type> Enriched_pixel;

          array<array<Enriched_pixel, 2>, 2> square =
            {{ {{ Enriched_pixel(pix00, Point_3(), Image_word_type()),
                  Enriched_pixel(pix01, Point_3(), Image_word_type()) }},
               {{ Enriched_pixel(pix10, Point_3(), Image_word_type()),
                  Enriched_pixel(pix11, Point_3(), Image_word_type()) }} }};

          std::map<Image_word_type, int> pixel_values_set;
          for(int ii = 0; ii < 2; ++ii)
            for(int jj = 0; jj < 2; ++jj)
            {
              const Pixel& pixel = get<0>(square[ii][jj]);
              double x = pixel[0] * vx;
              double y = pixel[1] * vy;
              double z = pixel[2] * vz;
              get<1>(square[ii][jj]) = Point_3(x, y, z);
              get<2>(square[ii][jj]) = static_cast<Image_word_type>(cgal_image.value(pixel[0],
                                                                                     pixel[1],
                                                                                     pixel[2]));
              ++pixel_values_set[get<2>(square[ii][jj])];
            }
          const Point_3& p00 = get<1>(square[0][0]);
          const Point_3& p10 = get<1>(square[1][0]);
          const Point_3& p01 = get<1>(square[0][1]);
          const Point_3& p11 = get<1>(square[1][1]);

          bool out00 = null(get<2>(square[0][0]));
          bool out10 = null(get<2>(square[1][0]));
          bool out01 = null(get<2>(square[0][1]));
          bool out11 = null(get<2>(square[1][1]));

          //
          // Protect the edges of the cube
          //
          if(pix00[axis_xx] == 0 &&
             ! ( out00 && out01 ) )
          {
            g_manip.try_add_edge(g_manip.get_vertex(p00),
                                 g_manip.get_vertex(p01));
          }
          if(pix11[axis_xx] == image_dims[axis_xx]-1 &&
             ! ( out10 && out11 ) )
          {
            g_manip.try_add_edge(g_manip.get_vertex(p10),
                                 g_manip.get_vertex(p11));
          }
          if(pix00[axis_yy] == 0 &&
             ! ( out00 && out10 ) )
          {
            g_manip.try_add_edge(g_manip.get_vertex(p00),
                                 g_manip.get_vertex(p10));
          }
          if(pix11[axis_yy] == image_dims[axis_yy]-1 &&
             ! ( out01 && out11 ) )
          {
            g_manip.try_add_edge(g_manip.get_vertex(p01),
                                 g_manip.get_vertex(p11));
          }

          //
          // Protect lines inside the square
          //
          switch(pixel_values_set.size()) {
          case 4: {
            CGAL_assertion(get<2>(square[0][0]) != get<2>(square[0][1]));
            CGAL_assertion(get<2>(square[0][0]) != get<2>(square[1][0]));
            CGAL_assertion(get<2>(square[0][0]) != get<2>(square[1][1]));
            CGAL_assertion(get<2>(square[1][0]) != get<2>(square[1][1]));
            CGAL_assertion(get<2>(square[0][1]) != get<2>(square[1][1]));
            CGAL_assertion(get<2>(square[0][1]) != get<2>(square[1][0]));
case_4:
            // case 4 or case 2-2
            ++case4;
            vertex_descriptor left   = g_manip.split(p00, p01, out00, out01);
            vertex_descriptor right  = g_manip.split(p10, p11, out10, out11);
            vertex_descriptor top    = g_manip.split(p01, p11, out01, out11);
            vertex_descriptor bottom = g_manip.split(p00, p10, out00, out10);
            vertex_descriptor vmid   = g_manip.get_vertex(midpoint(p00, p11));
            g_manip.try_add_edge(left   , vmid);
            g_manip.try_add_edge(right  , vmid);
            g_manip.try_add_edge(top    , vmid);
            g_manip.try_add_edge(bottom , vmid);
          }
            break;
          case 3: {
            if(get<2>(square[0][0]) == get<2>(square[1][1])) {
              // Diagonal, but the wrong one.
              // Vertical swap
              std::swap(square[0][1], square[0][0]); std::swap(out01, out00);
              std::swap(square[1][1], square[1][0]); std::swap(out11, out10);
            }
            if(get<2>(square[0][1]) == get<2>(square[1][0])) {
              // diagonal case 1-2-1
              CGAL_assertion(get<2>(square[0][1]) == get<2>(square[1][0]));
              CGAL_assertion(get<2>(square[1][1]) != get<2>(square[0][0]));
              CGAL_assertion(get<2>(square[0][1]) != get<2>(square[0][0]));
              CGAL_assertion(get<2>(square[0][1]) != get<2>(square[1][1]));
              ++case121;
              vertex_descriptor left   = g_manip.split(p00, p01, out00, out01);
              vertex_descriptor right  = g_manip.split(p10, p11, out10, out11);
              vertex_descriptor top    = g_manip.split(p01, p11, out01, out11);
              vertex_descriptor bottom = g_manip.split(p00, p10, out00, out10);

              vertex_descriptor old_left = left;
              vertex_descriptor old_right = right;
              vertex_descriptor v_int_left, v_int_right;

              // approximate the arcs by 10 segments
              //   -> 9 intermediate vertices
              for(double x = 0.05; x < 0.5; x+= 0.05)
              {
                const Point_3 inter_left =
                  p00
                  +      x                * (p10 - p00)  // x
                  + ((1.-2.*x)/(2.-3.*x)) * (p01 - p00); // y
                const Point_3 inter_right =
                  p11
                  +      x                * (p01 - p11)  // x
                  + ((1.-2.*x)/(2.-3.*x)) * (p10 - p11); // y
                v_int_left  = g_manip.get_vertex(inter_left);
                v_int_right = g_manip.get_vertex(inter_right);
                g_manip.try_add_edge(old_left,  v_int_left);
                g_manip.try_add_edge(old_right, v_int_right);
                old_left = v_int_left;
                old_right = v_int_right;
              }
              g_manip.try_add_edge(v_int_left,  bottom);
              g_manip.try_add_edge(v_int_right, top);
            } else {
              // case 2-1-1
              if(get<2>(square[0][0]) == get<2>(square[1][0])) {
                // Diagonal swap
                std::swap(square[0][1], square[1][0]); std::swap(out01, out10);
              } else
              if(get<2>(square[0][1]) == get<2>(square[1][1])) {
                // The other diagonal swap
                std::swap(square[0][0], square[1][1]); std::swap(out00, out11);
              } else
              if(get<2>(square[1][0]) == get<2>(square[1][1])) {
                // Vertical swap
                std::swap(square[0][0], square[1][0]); std::swap(out00, out10);
                std::swap(square[0][1], square[1][1]); std::swap(out01, out11);
              }
              CGAL_assertion(get<2>(square[0][0]) == get<2>(square[0][1]));
              CGAL_assertion(get<2>(square[0][0]) != get<2>(square[1][0]));
              CGAL_assertion(get<2>(square[0][0]) != get<2>(square[1][1]));
              CGAL_assertion(get<2>(square[1][0]) != get<2>(square[1][1]));
              ++case211;
              Point_3 midleft =  midpoint(p00, p01);
              Point_3 midright = midpoint(p10, p11);
              Point_3 inter = midleft + (2./3)*(midright-midleft);
              vertex_descriptor v_inter = g_manip.get_vertex(inter);
              vertex_descriptor right  = g_manip.split(p10, p11, out10, out11);
              vertex_descriptor top    = g_manip.split(p01, p11, out01, out11);
              vertex_descriptor bottom = g_manip.split(p00, p10, out00, out10);

              vertex_descriptor old_top = top;
              vertex_descriptor old_bottom = bottom;
              vertex_descriptor v_int_top, v_int_bottom;

              // approximate the arcs by 10 segments
              //   -> 9 intermediate vertices
              for(double x = 0.51666; x < 0.66; x+= 0.016666)
              {
                const Point_3 inter_top =
                  p00
                  +      x         * (p10 - p00)  // x
                  + ((1./x) - 1.)  * (p01 - p00); // y
                const Point_3 inter_bottom =
                  p00
                  +      x         * (p10 - p00)  // x
                  + (2.-(1./x))    * (p01 - p00); // y
                v_int_top    = g_manip.get_vertex(inter_top);
                v_int_bottom = g_manip.get_vertex(inter_bottom);
                g_manip.try_add_edge(old_top,    v_int_top);
                g_manip.try_add_edge(old_bottom, v_int_bottom);
                old_top = v_int_top;
                old_bottom = v_int_bottom;
              }

              g_manip.try_add_edge(v_int_bottom,    v_inter);
              g_manip.try_add_edge(v_int_top, v_inter);
              g_manip.try_add_edge(right,  v_inter);
            } // end case 2-1-1
          } // end `case 3:`
            break;
          case 2: {
            if(pixel_values_set.begin()->second ==
               pixel_values_set.rbegin()->second)
            {
              // Case of two colors with two pixels each.

              if(get<2>(square[0][0])==get<2>(square[1][0])) {
                // case 2-2, diagonal swap
                std::swap(square[0][1], square[1][0]); std::swap(out01, out10);
                CGAL_assertion(get<2>(square[0][0])==get<2>(square[0][1]));
              }
              if(get<2>(square[1][0])==get<2>(square[1][1])) {
                // case 2-2, vertical swap
                std::swap(square[0][1], square[1][1]); std::swap(out01, out11);
                std::swap(square[0][0], square[1][0]); std::swap(out00, out10);
                CGAL_assertion(get<2>(square[0][0])==get<2>(square[0][1]));
              }
              if(get<2>(square[0][1])==get<2>(square[1][1])) {
                // case 2-2, diagonal swap
                std::swap(square[0][0], square[1][1]); std::swap(out00, out11);
                CGAL_assertion(get<2>(square[0][0])==get<2>(square[0][1]));
              }

              if(get<2>(square[0][0])==get<2>(square[0][1])) {
                // vertical case 2-2
                ++case22;
                CGAL_assertion(get<2>(square[1][0])==get<2>(square[1][1]));
                CGAL_assertion(get<2>(square[1][0])!=get<2>(square[0][1]));
                vertex_descriptor top    = g_manip.split(p01, p11, out01, out11);
                vertex_descriptor bottom = g_manip.split(p00, p10, out00, out10);
                g_manip.try_add_edge(top, bottom);
              } else {
                // Else diagonal case case 2-2
                // Same as the case with 4 colors
                CGAL_assertion(get<2>(square[0][0])==get<2>(square[1][1]));
                CGAL_assertion(get<2>(square[1][0])==get<2>(square[0][1]));
                CGAL_assertion(get<2>(square[0][0])!=get<2>(square[0][1]));
                goto case_4;
              }
            }
            else {
              // case of two colors with one pixel green and three red
              Image_word_type value_alone;
              if(pixel_values_set.begin()->second == 1) {
                value_alone = pixel_values_set.begin()->first;
              } else {
                CGAL_assertion(pixel_values_set.begin()->second == 3);
                CGAL_assertion(pixel_values_set.rbegin()->second ==1);
                value_alone = pixel_values_set.rbegin()->first;
              }
              if(get<2>(square[0][1]) == value_alone) {
                // central symmetry
                std::swap(square[0][1], square[1][0]); std::swap(out01, out10);
                std::swap(square[0][0], square[1][1]); std::swap(out00, out11);
                CGAL_assertion(get<2>(square[1][0]) == value_alone);
              }
              if(get<2>(square[1][1]) == value_alone) {
                // vertical swap
                std::swap(square[0][0], square[0][1]); std::swap(out00, out01);
                std::swap(square[1][0], square[1][1]); std::swap(out10, out11);
                CGAL_assertion(get<2>(square[1][0]) == value_alone);
              }
              if(get<2>(square[0][0]) == value_alone) {
                // horizontal swap
                std::swap(square[0][1], square[1][1]); std::swap(out01, out11);
                std::swap(square[0][0], square[1][0]); std::swap(out00, out10);
                CGAL_assertion(get<2>(square[1][0]) == value_alone);
              }
              ++case31;
              CGAL_assertion(get<2>(square[1][0]) == value_alone);
              CGAL_assertion(get<2>(square[1][0]) != get<2>(square[0][0]));
              CGAL_assertion(get<2>(square[1][1]) == get<2>(square[0][0]));
              CGAL_assertion(get<2>(square[0][1]) == get<2>(square[0][0]));
              vertex_descriptor bottom = g_manip.split(p00, p10, out00, out10);
              vertex_descriptor old = bottom;

              vertex_descriptor v_int;
              for(double x = 0.55; x < 1.; x+= 0.05)
              {
                const Point_3 inter =
                  p00
                  +      x         * (p10 - p00)  // x
                  + (1.-1./(2.*x)) * (p01 - p00); // y
                v_int = g_manip.get_vertex(inter);
                g_manip.try_add_edge(old, v_int);
                old = v_int;
              }

              vertex_descriptor right  = g_manip.split(p10, p11, out10, out11);
              g_manip.try_add_edge(v_int, right);
            }
          }
            break;
          default: // case 1
            ++case1;
            // nothing to do
            break;
          }
        }
  }
  // std::cerr << "case 4:     " << case4 << std::endl;
  // std::cerr << "case 2-1-1: " << case211 << std::endl;
  // std::cerr << "case 1-2-1: " << case121 << std::endl;
  // std::cerr << "case 3-1:   " << case31 << std::endl;
  // std::cerr << "case 2-2:   " << case22 << std::endl;
  // std::cerr << "case 1:     " << case1 << std::endl;

  const std::ptrdiff_t nb_facets =
    case4 + case211 + case121 + case31 + case22 + case1;
  const std::ptrdiff_t expected_nb_facets =
    2*((xdim-1)*(ydim-1) + (ydim-1)*(zdim-1) + (xdim-1)*(zdim-1));

  // std::cerr << "nb of facets:           " << nb_facets << std::endl
  //           << " expected nb of facets: " << expected_nb_facets << std::endl;

  CGAL_assertion(nb_facets == expected_nb_facets);
  CGAL_USE(nb_facets); CGAL_USE(expected_nb_facets);

  snap_graph_vertices(graph,
                      vx, vy, vz,
                      existing_polylines_begin, existing_polylines_end,
                      K());

  Mesh_3::Polyline_visitor<Point_3, Graph> visitor(polylines, graph);
  split_graph_into_polylines(graph, visitor);
}

template <typename P,
          typename PolylineInputIterator>
void
polylines_to_protect(std::vector<std::vector<P> >& polylines,
                     PolylineInputIterator existing_polylines_begin,
                     PolylineInputIterator existing_polylines_end)
{
  typedef P Point_3;
  typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, Point_3> Graph;
  typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
  typedef typename std::iterator_traits<PolylineInputIterator>::value_type Polyline;

  Graph graph;
  internal::Mesh_3::Graph_manipulations<Graph, Point_3> g_manip(graph);

  for (PolylineInputIterator poly_it = existing_polylines_begin;
       poly_it != existing_polylines_end; ++poly_it)
  {
    Polyline polyline = *poly_it;
    if (polyline.size() < 2)
      continue;

    typename Polyline::iterator pit = polyline.begin();
    while (boost::next(pit) != polyline.end())
    {
      vertex_descriptor v = g_manip.get_vertex(*pit);
      vertex_descriptor w = g_manip.get_vertex(*boost::next(pit));
      g_manip.try_add_edge(v, w);
      ++pit;
    }
  }

  Mesh_3::Polyline_visitor<Point_3, Graph> visitor(polylines, graph);
  split_graph_into_polylines(graph, visitor);
}

template <typename P, typename Image_word_type, typename Null_subdomain_index>
void
polylines_to_protect(const CGAL::Image_3& cgal_image,
                     std::vector<std::vector<P> >& polylines,
                     Image_word_type* word_type,
                     Null_subdomain_index null)
{
  polylines_to_protect<P>
    (cgal_image,
     cgal_image.vx(), cgal_image.vy(),cgal_image.vz(),
     polylines,
     word_type,
     null,
     0,
     0);
}

template <typename P, typename Image_word_type>
void
polylines_to_protect(const CGAL::Image_3& cgal_image,
                     std::vector<std::vector<P> >& polylines)
{
  polylines_to_protect<P>
    (cgal_image,
     cgal_image.vx(), cgal_image.vy(),cgal_image.vz(),
     polylines,
     (Image_word_type*)0,
     CGAL::Null_subdomain_index(),
     0,
     0);
}


template <typename P,
          typename Image_word_type,
          typename PolylineInputIterator>
void
polylines_to_protect(const CGAL::Image_3& cgal_image,
                     std::vector<std::vector<P> >& polylines,
                     PolylineInputIterator existing_polylines_begin,
                     PolylineInputIterator existing_polylines_end)
{
  polylines_to_protect<P>
    (cgal_image,
     cgal_image.vx(), cgal_image.vy(),cgal_image.vz(),
     polylines,
     (Image_word_type*)0,
     CGAL::Null_subdomain_index(),
     existing_polylines_begin,
     existing_polylines_end);
}

template <typename P,
          typename Image_word_type,
          typename Null_subdomain_index,
          typename PolylineInputIterator>
void
polylines_to_protect(const CGAL::Image_3& cgal_image,
                     std::vector<std::vector<P> >& polylines,
                     Image_word_type* word_type,
                     Null_subdomain_index null,
                     PolylineInputIterator existing_polylines_begin,
                     PolylineInputIterator existing_polylines_end)
{
  polylines_to_protect<P>
    (cgal_image,
     cgal_image.vx(), cgal_image.vy(),cgal_image.vz(),
     polylines,
     word_type,
     null,
     existing_polylines_begin,
     existing_polylines_end);
}

} // namespace CGAL

#endif // CGAL_MESH_3_POLYLINES_TO_PROTECT_H
