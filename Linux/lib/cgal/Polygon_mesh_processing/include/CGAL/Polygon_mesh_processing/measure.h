// Copyright (c) 2015 GeometryFactory (France).
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
// Author(s)     : Andreas Fabri

#ifndef CGAL_POLYGON_MESH_PROCESSING_MEASURE_H
#define CGAL_POLYGON_MESH_PROCESSING_MEASURE_H

#include <CGAL/boost/graph/iterator.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/properties.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/mpl/eval_if.hpp>
#include <boost/mpl/identity.hpp>
#include <boost/type_traits/is_base_of.hpp>

#include <CGAL/Polygon_mesh_processing/internal/named_function_params.h>
#include <CGAL/Polygon_mesh_processing/internal/named_params_helper.h>

#ifdef DOXYGEN_RUNNING
#define CGAL_PMP_NP_TEMPLATE_PARAMETERS NamedParameters
#define CGAL_PMP_NP_CLASS NamedParameters
#endif

namespace CGAL {

namespace Polygon_mesh_processing {

  /**
  * \ingroup measure_grp
  * computes the length of an edge of a given polygon mesh.
  * The edge is given by one of its halfedges, or the edge itself.
  *
  * @tparam PolygonMesh a model of `HalfedgeGraph` that has an internal property map
  *         for `CGAL::vertex_point_t`
  * @tparam NamedParameters a sequence of \ref namedparameters
  *
  * @param h one halfedge of the edge to compute the length
  * @param pmesh the polygon mesh to which `h` belongs
  * @param np optional sequence of \ref namedparameters among the ones listed below
  *
  * \cgalNamedParamsBegin
  *    \cgalParamBegin{vertex_point_map} the property map with the points associated to the vertices of `pmesh` \cgalParamEnd
  *    \cgalParamBegin{geom_traits} an instance of a geometric traits class, model of `Kernel`\cgalParamEnd
  * \cgalNamedParamsEnd
  *
  * @return the length of `h`. The return type `FT` is a number type. It is
  * either deduced from the `geom_traits` \ref namedparameters if provided,
  * or the geometric traits class deduced from the point property map
  * of `pmesh`.
  *
  * \warning This function involves a square root computation.
  * If `FT` does not have a `sqrt()` operation, the square root computation
  * will be done approximately.
  *
  * @sa `face_border_length()`
  */
  template<typename PolygonMesh,
           typename NamedParameters>
#ifdef DOXYGEN_RUNNING
  FT
#else
  typename GetGeomTraits<PolygonMesh, NamedParameters>::type::FT
#endif
  edge_length(typename boost::graph_traits<PolygonMesh>::halfedge_descriptor h
              , const PolygonMesh& pmesh
              , const NamedParameters& np)
  {
    using boost::choose_const_pmap;
    using boost::get_param;

    typename GetVertexPointMap<PolygonMesh, NamedParameters>::const_type
    vpm = choose_const_pmap(get_param(np, CGAL::vertex_point),
                            pmesh,
                            CGAL::vertex_point);

    return CGAL::approximate_sqrt(CGAL::squared_distance(get(vpm, source(h, pmesh)),
                                                         get(vpm, target(h, pmesh))));
  }

  template<typename PolygonMesh>
  typename CGAL::Kernel_traits<typename property_map_value<PolygonMesh,
    CGAL::vertex_point_t>::type>::Kernel::FT
  edge_length(typename boost::graph_traits<PolygonMesh>::halfedge_descriptor h
            , const PolygonMesh& pmesh)
  {
    return edge_length(h, pmesh,
      CGAL::Polygon_mesh_processing::parameters::all_default());
  }
  // edge overloads
  template<typename PolygonMesh,
           typename NamedParameters>
  typename GetGeomTraits<PolygonMesh, NamedParameters>::type::FT
  edge_length(typename boost::graph_traits<PolygonMesh>::edge_descriptor e
              , const PolygonMesh& pmesh
              , const NamedParameters& np)
  {
    return edge_length(halfedge(e,pmesh), pmesh, np);
  }

  template<typename PolygonMesh>
  typename CGAL::Kernel_traits<typename property_map_value<PolygonMesh,
    CGAL::vertex_point_t>::type>::Kernel::FT
  edge_length(typename boost::graph_traits<PolygonMesh>::edge_descriptor e
            , const PolygonMesh& pmesh)
  {
    return edge_length(halfedge(e,pmesh), pmesh);
  }

  /**
  * \ingroup measure_grp
  * computes the length of the border polyline
  * that contains a given halfedge.
  *
  * @tparam PolygonMesh a model of `HalfedgeGraph` that has an internal property map
  *         for `CGAL::vertex_point_t`
  * @tparam NamedParameters a sequence of \ref namedparameters
  *
  * @param h a halfedge of the border polyline of which the length is computed
  * @param pmesh the polygon mesh to which `h` belongs
  * @param np optional sequence of \ref namedparameters among the ones listed below
  *
  * \cgalNamedParamsBegin
  *    \cgalParamBegin{vertex_point_map} the property map with the points associated to the vertices of `pmesh` \cgalParamEnd
*    \cgalParamBegin{geom_traits} an instance of a geometric traits class, model of `Kernel`\cgalParamEnd
  * \cgalNamedParamsEnd
  *
  * @return the length of the sequence of border edges of `face(h, pmesh)`.
  * The return type `FT` is a number type. It is
  * either deduced from the `geom_traits` \ref namedparameters if provided,
  * or the geometric traits class deduced from the point property map
  * of `pmesh`.
  *
  * \warning This function involves a square root computation.
  * If `Kernel::FT` does not have a `sqrt()` operation, the square root computation
  * will be done approximately.
  *
  * @sa `edge_length()`
  */
  template<typename PolygonMesh,
           typename NamedParameters>
#ifdef DOXYGEN_RUNNING
   FT
#else
  typename GetGeomTraits<PolygonMesh, NamedParameters>::type::FT
#endif
  face_border_length(
              typename boost::graph_traits<PolygonMesh>::halfedge_descriptor h
              , const PolygonMesh& pmesh
              , const NamedParameters& np)
  {
    typename GetGeomTraits<PolygonMesh, NamedParameters>::type::FT result = 0.;
    BOOST_FOREACH(typename boost::graph_traits<PolygonMesh>::halfedge_descriptor haf,
                  halfedges_around_face(h, pmesh))
    {
      result += edge_length(haf, pmesh, np);
      exact(result);
    }
    return result;
  }

  template<typename PolygonMesh>
  typename CGAL::Kernel_traits<typename property_map_value<PolygonMesh,
    CGAL::vertex_point_t>::type>::Kernel::FT
  face_border_length(
              typename boost::graph_traits<PolygonMesh>::halfedge_descriptor h
              , const PolygonMesh& pmesh)
  {
    return face_border_length(h, pmesh,
      CGAL::Polygon_mesh_processing::parameters::all_default());
  }


  /**
  * \ingroup measure_grp
  * computes the area of a face of a given
  * triangulated surface mesh.
  *
  * @tparam TriangleMesh a model of `HalfedgeGraph` that has an internal property map
  *         for `CGAL::vertex_point_t`
  * @tparam NamedParameters a sequence of \ref namedparameters
  *
  * @param f the face of which the area is computed
  * @param tmesh the triangulated surface mesh to which `f` belongs
  * @param np optional sequence of \ref namedparameters among the ones listed below
  *
  * \cgalNamedParamsBegin
  *    \cgalParamBegin{vertex_point_map} the property map with the points associated to the vertices of `pmesh` \cgalParamEnd
  *  \cgalParamBegin{geom_traits} an instance of a geometric traits class, model of `Kernel`\cgalParamEnd
  * \cgalNamedParamsEnd
  *
  *@pre `f != boost::graph_traits<TriangleMesh>::%null_face()`
  *
  * @return the area of `f`.
  * The return type `FT` is a number type. It is
  * either deduced from the `geom_traits` \ref namedparameters if provided,
  * or the geometric traits class deduced from the point property map
  * of `tmesh`.
  *
  * @sa `area()`
  */
  template<typename TriangleMesh,
           typename CGAL_PMP_NP_TEMPLATE_PARAMETERS>
#ifdef DOXYGEN_RUNNING
  FT
#else
  typename GetGeomTraits<TriangleMesh, CGAL_PMP_NP_CLASS>::type::FT
#endif
  face_area(typename boost::graph_traits<TriangleMesh>::face_descriptor f
            , const TriangleMesh& tmesh
            , const CGAL_PMP_NP_CLASS& np)
  {
    using boost::choose_const_pmap;
    using boost::get_param;

    CGAL_precondition(boost::graph_traits<TriangleMesh>::null_face() != f);

    typename GetVertexPointMap<TriangleMesh, CGAL_PMP_NP_CLASS>::const_type
    vpm = choose_const_pmap(get_param(np, CGAL::vertex_point),
                            tmesh,
                            CGAL::vertex_point);

    typedef typename boost::graph_traits<TriangleMesh>::halfedge_descriptor halfedge_descriptor;
    halfedge_descriptor hd = halfedge(f, tmesh);
    halfedge_descriptor nhd = next(hd, tmesh);

    typename GetGeomTraits<TriangleMesh, CGAL_PMP_NP_CLASS>::type traits;

    return approximate_sqrt(
             traits.compute_squared_area_3_object()(get(vpm, source(hd, tmesh)),
                                                    get(vpm, target(hd, tmesh)),
                                                    get(vpm, target(nhd, tmesh))));
  }

  template<typename TriangleMesh>
  typename CGAL::Kernel_traits<typename property_map_value<TriangleMesh,
    CGAL::vertex_point_t>::type>::Kernel::FT
  face_area(typename boost::graph_traits<TriangleMesh>::face_descriptor f
            , const TriangleMesh& tmesh)
  {
    return face_area(f, tmesh,
      CGAL::Polygon_mesh_processing::parameters::all_default());
  }

  /**
  * \ingroup measure_grp
  * computes the area of a range of faces of a given
  * triangulated surface mesh.
  *
  * @tparam FaceRange range of `boost::graph_traits<PolygonMesh>::%face_descriptor`,
          model of `Range`.
          Its iterator type is `InputIterator`.
  * @tparam TriangleMesh a model of `HalfedgeGraph` that has an internal property map
  *         for `CGAL::vertex_point_t`
  * @tparam NamedParameters a sequence of \ref namedparameters
  *
  * @param face_range the range of faces of which the area is computed
  * @param tmesh the triangulated surface mesh to which the faces of `face_range` belong
  * @param np optional sequence of \ref namedparameters among the ones listed below
  *
  * \cgalNamedParamsBegin
  *    \cgalParamBegin{vertex_point_map} the property map with the points associated to the vertices of `pmesh` \cgalParamEnd
  *  \cgalParamBegin{geom_traits} an instance of a geometric traits class, model of `Kernel` \cgalParamEnd
  * \cgalNamedParamsEnd
  *
  * @return sum of face areas of `faces`.
  * The return type `FT` is a number type. It is
  * either deduced from the `geom_traits` \ref namedparameters if provided,
  * or the geometric traits class deduced from the point property map
  * of `tmesh`.
  *
  * \warning This function involves a square root computation.
  * If `Kernel::FT` does not have a `sqrt()` operation, the square root computation
  * will be done approximately.
  *
  * @sa `face_area()`
  */
  template<typename FaceRange,
           typename TriangleMesh,
           typename CGAL_PMP_NP_TEMPLATE_PARAMETERS>
#ifdef DOXYGEN_RUNNING
  FT
#else
  typename GetGeomTraits<TriangleMesh, CGAL_PMP_NP_CLASS>::type::FT
#endif
  area(FaceRange face_range
     , const TriangleMesh& tmesh
     , const CGAL_PMP_NP_CLASS& np)
  {
    typedef typename boost::graph_traits<TriangleMesh>::face_descriptor face_descriptor;
    typename GetGeomTraits<TriangleMesh, CGAL_PMP_NP_CLASS>::type::FT result = 0.;
    BOOST_FOREACH(face_descriptor f, face_range)
    {
      result += face_area(f, tmesh, np);
      exact(result);
    }
    return result;
  }

  template<typename FaceRange, typename TriangleMesh>
  typename CGAL::Kernel_traits<
    typename boost::mpl::eval_if<
      boost::is_base_of<CGAL::named_params_base, TriangleMesh>,
      boost::mpl::identity<TriangleMesh>,
      property_map_value<TriangleMesh, CGAL::vertex_point_t>
    >::type
  >::Kernel::FT
  area(FaceRange face_range, const TriangleMesh& tmesh)
  {
    return area(face_range, tmesh,
                CGAL::Polygon_mesh_processing::parameters::all_default());
  }

  /**
  * \ingroup measure_grp
  * computes the surface area of a triangulated surface mesh.
  *
  * @tparam TriangleMesh a model of `HalfedgeGraph` that has an internal property map
  *         for `CGAL::vertex_point_t`
  * @tparam NamedParameters a sequence of \ref namedparameters
  *
  * @param tmesh the triangulated surface mesh
  * @param np optional sequence of \ref namedparameters among the ones listed below
  *
  * \cgalNamedParamsBegin
  *    \cgalParamBegin{vertex_point_map} the property map with the points associated to the vertices of `pmesh` \cgalParamEnd
  *  \cgalParamBegin{geom_traits}an instance of a geometric traits class, model of `Kernel`\cgalParamEnd
  * \cgalNamedParamsEnd
  *
  * @return the surface area of `tmesh`.
  * The return type `FT` is a number type. It is
  * either deduced from the `geom_traits` \ref namedparameters if provided,
  * or the geometric traits class deduced from the point property map
  * of `tmesh`.
  *
  * \warning This function involves a square root computation.
  * If `Kernel::FT` does not have a `sqrt()` operation, the square root computation
  * will be done approximately.
  *
  * @sa `face_area()`
  */
  template<typename TriangleMesh
         , typename CGAL_PMP_NP_TEMPLATE_PARAMETERS>
#ifdef DOXYGEN_RUNNING
  FT
#else
  typename GetGeomTraits<TriangleMesh, CGAL_PMP_NP_CLASS>::type::FT
#endif
  area(const TriangleMesh& tmesh, const CGAL_PMP_NP_CLASS& np)
  {
    return area(faces(tmesh), tmesh, np);
  }

  template<typename TriangleMesh>
  typename CGAL::Kernel_traits<typename property_map_value<TriangleMesh,
    CGAL::vertex_point_t>::type>::Kernel::FT
  area(const TriangleMesh& tmesh)
  {
    return area(faces(tmesh), tmesh
      , CGAL::Polygon_mesh_processing::parameters::all_default());
  }

  /**
  * \ingroup measure_grp
  * computes the volume of the domain bounded by
  * a closed triangulated surface mesh.
  *
  * @tparam TriangleMesh a model of `HalfedgeGraph` that has an internal property map
  *         for `CGAL::vertex_point_t`
  * @tparam NamedParameters a sequence of \ref namedparameters
  *
  * @param tmesh the closed triangulated surface mesh bounding the volume
  * @param np optional sequence of \ref namedparameters among the ones listed below
  *
  * @pre `tmesh` is closed
  *
  * \cgalNamedParamsBegin
  *  \cgalParamBegin{vertex_point_map} the property map with the points associated to the vertices of `pmesh` \cgalParamEnd
  *  \cgalParamBegin{geom_traits}an instance of a geometric traits class, model of `Kernel`\cgalParamEnd
  * \cgalNamedParamsEnd
  *
  * @return the volume bounded by `tmesh`.
  * The return type `FT` is a number type. It is
  * either deduced from the `geom_traits` \ref namedparameters if provided,
  * or the geometric traits class deduced from the point property map
  * of `tmesh`.
  */
  template<typename TriangleMesh
         , typename CGAL_PMP_NP_TEMPLATE_PARAMETERS>
#ifdef DOXYGEN_RUNNING
  FT
#else
  typename GetGeomTraits<TriangleMesh, CGAL_PMP_NP_CLASS>::type::FT
#endif
  volume(const TriangleMesh& tmesh, const CGAL_PMP_NP_CLASS& np)
  {
    CGAL_assertion(is_triangle_mesh(tmesh));
    CGAL_assertion(is_closed(tmesh));

    using boost::choose_const_pmap;
    using boost::get_param;

    typename GetVertexPointMap<TriangleMesh, CGAL_PMP_NP_CLASS>::const_type
      vpm = choose_const_pmap(get_param(np, CGAL::vertex_point),
                              tmesh,
                              CGAL::vertex_point);
    typename GetGeomTraits<TriangleMesh, CGAL_PMP_NP_CLASS>::type::Point_3
      origin(0, 0, 0);

    typedef typename boost::graph_traits<TriangleMesh>::face_descriptor face_descriptor;

    typename GetGeomTraits<TriangleMesh, CGAL_PMP_NP_CLASS>::type::FT volume = 0.;
    BOOST_FOREACH(face_descriptor f, faces(tmesh))
    {
      volume += CGAL::volume(origin,
        get(vpm, target(halfedge(f, tmesh), tmesh)),
        get(vpm, target(next(halfedge(f, tmesh), tmesh), tmesh)),
        get(vpm, target(prev(halfedge(f, tmesh), tmesh), tmesh)));
      exact(volume);
    }
    return volume;
  }

  template<typename TriangleMesh>
  typename CGAL::Kernel_traits<typename property_map_value<TriangleMesh,
    CGAL::vertex_point_t>::type>::Kernel::FT
  volume(const TriangleMesh& tmesh)
  {
    return volume(tmesh,
      CGAL::Polygon_mesh_processing::parameters::all_default());
  }

}
}

#endif // CGAL_POLYGON_MESH_PROCESSING_MEASURE_H
