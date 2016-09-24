// Copyright (c) 2003-2007  INRIA Sophia-Antipolis (France).
// Copyright (c) 2008,2013  GeometryFactory, Sophia Antipolis (France)
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
// Author(s)     : Steve Oudot, David Rey, Mariette Yvinec, Laurent Rineau, Andreas Fabri

#ifndef CGAL_MESH_3_REFINE_FACETS_MANIFOLD_BASE_H
#define CGAL_MESH_3_REFINE_FACETS_MANIFOLD_BASE_H

#include <CGAL/utility.h>
#include <set>
#include <vector>
#include <boost/bimap.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <boost/foreach.hpp>
#include <boost/mpl/has_xxx.hpp>
#include <CGAL/Mesh_facet_topology.h>

namespace CGAL {

namespace Mesh_3 {

BOOST_MPL_HAS_XXX_TRAIT_DEF(Has_manifold_criterion)

template <typename Criteria,
          bool has_Has_manifold_criterion =
          has_Has_manifold_criterion<Criteria>::value>
struct Has_manifold_criterion :
    public CGAL::Boolean_tag<Criteria::Has_manifold_criterion::value>
// when Criteria has the nested type Has_manifold_criterion
{};

template <typename Criteria>
struct Has_manifold_criterion<Criteria, false> : public CGAL::Tag_false
// when Criteria does not have the nested type Has_manifold_criterion
{};



template <typename Criteria>
bool get_with_manifold(const Criteria&, CGAL::Tag_false)
{
  return false;
}

template <typename Criteria>
bool get_with_manifold(const Criteria& c, CGAL::Tag_true)
{
  return (c.topology() & MANIFOLD_WITH_BOUNDARY) != 0;
}

template <typename Criteria>
bool get_with_manifold(const Criteria& c)
{
  return get_with_manifold(c, Has_manifold_criterion<Criteria>());
}

template <typename Criteria>
bool get_with_boundary(const Criteria&, CGAL::Tag_false)
{
  return false;
}

template <typename Criteria>
bool get_with_boundary(const Criteria& c, CGAL::Tag_true)
{
  return (c.topology() & NO_BOUNDARY) == 0;
}

template <typename Criteria>
bool get_with_boundary(const Criteria& c)
{
  return get_with_boundary(c, Has_manifold_criterion<Criteria>());
}

template<class Tr,
         class Criteria,
         class MeshDomain,
         class Complex3InTriangulation3,
         class Concurrency_tag,
         class Container_
         >
class Refine_facets_manifold_base
  : public Refine_facets_3_base<Tr,
                                Criteria,
                                MeshDomain,
                                Complex3InTriangulation3,
                                Concurrency_tag,
                                Container_>
{
  typedef Refine_facets_3_base<Tr,
                               Criteria,
                               MeshDomain,
                               Complex3InTriangulation3,
                               Concurrency_tag,
                               Container_> Base;

public:
  typedef Complex3InTriangulation3 C3t3;
  typedef MeshDomain Mesh_domain;
  typedef typename Tr::Point Point;
  typedef typename Tr::Facet Facet;
  typedef typename Tr::Vertex_handle Vertex_handle;

  typedef typename Triangulation_mesher_level_traits_3<Tr>::Zone Zone;

protected:
  typedef typename Tr::Geom_traits GT;
  typedef typename GT::FT FT;
  typedef typename Tr::Edge Edge;
  typedef typename Tr::Cell_handle Cell_handle;

  typedef typename Tr::Facet_circulator Tr_facet_circulator;
  typedef std::pair<Vertex_handle, Vertex_handle> EdgeVV;

protected:
  typedef ::boost::bimap< EdgeVV,
                          ::boost::bimaps::multiset_of<int> > Bad_edges;
  typedef typename Bad_edges::value_type Bad_edge;

  mutable Bad_edges m_bad_edges;
  mutable std::set<Vertex_handle> m_bad_vertices;

  mutable bool m_manifold_info_initialized;
  mutable bool m_bad_vertices_initialized;
  bool m_with_manifold_criterion;
  bool m_with_boundary;

private:
  // computes and return an ordered pair of Vertex
  EdgeVV make_edgevv(const Vertex_handle vh1,
                     const Vertex_handle vh2) const {
    if (vh1 < vh2) {
      return std::make_pair(vh1, vh2);
    }
    else {
      return std::make_pair(vh2, vh1);
    }
  }

  // computes and returns the Edge type object from the EdgeVV object
  // use tr.is_edge(...)
  Edge edgevv_to_edge(const EdgeVV& arete) const {
    Vertex_handle v1 = arete.first;
    Vertex_handle v2 = arete.second;
    Cell_handle c;
    int index1=0, index2=0; // initialize to avoid g++ 4.8 -Wmaybe-uninitialized

    CGAL_assume_code(bool is_edge =)
    this->r_tr_.is_edge(v1, v2, c, index1, index2);
    CGAL_assume(is_edge);

    return make_triple( c, index1, index2 );
  }

  // computes and returns the EdgeVV type object from the Edge object
  EdgeVV edge_to_edgevv(const Edge& arete) const {
    return make_edgevv(arete.first->vertex(arete.second),
                       arete.first->vertex(arete.third) );
  }

  FT compute_distance_to_facet_center(const Facet& f,
                                      const Vertex_handle v) const {
    const Point& fcenter = f.first->get_facet_surface_center(f.second);
    const Point& vpoint = v->point();

    return
      this->r_tr_.geom_traits().compute_squared_distance_3_object()(fcenter.point(),
                                                              vpoint.point())
      - vpoint.weight();
  }

  Facet
  biggest_incident_facet_in_complex(const Vertex_handle v) const
  {
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
    std::cerr << "Bad vertex: " << v->point() << std::endl;
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS
    std::vector<Facet> facets;
    facets.reserve(64);
    this->r_tr_.incident_facets(v, std::back_inserter(facets));

    typename std::vector<Facet>::iterator fit = facets.begin();
    while(fit != facets.end() && !this->r_c3t3_.is_in_complex(*fit)) ++fit;
    CGAL_assertion(fit!=facets.end());
    CGAL_assertion_code(std::size_t facet_counter = 1);

#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
    std::cerr << "  " << v->cached_number_of_incident_facets()
              << " incident faces, with sizes:\n";
    std::cerr << "    " << compute_distance_to_facet_center(*fit, v) << "\n";
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS
    Facet biggest_facet = *fit++;
    while(fit != facets.end() && !this->r_c3t3_.is_in_complex(*fit)) ++fit;

    for (; fit != facets.end(); )
    {
      CGAL_assertion_code(++facet_counter);
      Facet current_facet = *fit;
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
      std::cerr << "    " << compute_distance_to_facet_center(*fit, v) << "\n";
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS
      // is the current facet bigger than the current biggest one
      if ( compute_distance_to_facet_center(current_facet, v) >
           compute_distance_to_facet_center(biggest_facet, v) )
      {
        biggest_facet = current_facet;
      }
      ++fit;
      while(fit != facets.end() && !this->r_c3t3_.is_in_complex(*fit)) ++fit;
    }
    CGAL_assertion(v->cached_number_of_incident_facets() ==
                   facet_counter);
    CGAL_assertion(this->r_c3t3_.is_in_complex(biggest_facet));
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
    std::cerr << "Biggest facet radius: "
              << compute_distance_to_facet_center(biggest_facet, v)
              << std::endl;
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS

    return biggest_facet;
  }

  Facet biggest_incident_facet_in_complex(const Edge& arete) const {
    // Find the first facet in the incident facets
    // of the edge which is in the Complex
    // use the list of incident facets in the complex
    Vertex_handle fev = edge_to_edgevv(arete).first;
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
    std::cerr << "Bad edge: (" << fev->point()
              << ", " << arete.first->vertex(arete.third)->point()
              << ")\n  incident facets sizes:\n";
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS
    Tr_facet_circulator fcirc = this->r_tr_.incident_facets(arete);
    while(!this->r_c3t3_.is_in_complex(*fcirc)) ++fcirc;
    Facet first_facet = *fcirc;
    Facet biggest_facet = *fcirc;
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
    std::cerr << "    "
              << compute_distance_to_facet_center(*fcirc, fev) << std::endl;
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS

    for (++fcirc; *fcirc != first_facet; ++fcirc) 
    {
      while(!this->r_c3t3_.is_in_complex(*fcirc)) ++fcirc;
      if(*fcirc == first_facet) break;
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
      std::cerr << "    "
                << compute_distance_to_facet_center(*fcirc, fev) << std::endl;
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS

      // is the current facet bigger than the current biggest one
      if ( compute_distance_to_facet_center(*fcirc, fev) >
           compute_distance_to_facet_center(biggest_facet,
                                            fev) ) {
        biggest_facet = *fcirc;
      }
    }
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
    std::cerr << "Biggest facet radius: "
              << compute_distance_to_facet_center(biggest_facet, fev)
              << std::endl;
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS

    return biggest_facet;
  }

  ///////////////////////
  // For before_insertion

  // Actions to perform on a facet inside the conflict zone
  void
  before_insertion_handle_facet_inside_conflict_zone (const Facet& f) 
  {
    if ( this->r_c3t3_.is_in_complex(f) ) {
      // foreach edge of f
      const Cell_handle cell = f.first;
      const int i = f.second;
      for(int j = 0; j < 3; ++j)
      {
        const int edge_index_va = this->r_tr_.vertex_triple_index(i, j);
        const int edge_index_vb = this->r_tr_.vertex_triple_index(i, (j == 2) ? 0 : (j+1));
        const Vertex_handle edge_va = cell->vertex(edge_index_va);
        const Vertex_handle edge_vb = cell->vertex(edge_index_vb);
        m_bad_edges.left.erase( make_edgevv(edge_va, edge_vb));
      }
    }
  }

  // Action to perform on a facet on the boundary of the conflict zone
  void
  before_insertion_handle_facet_on_boundary_of_conflict_zone(const Facet& f)
  {
    // perform the same operations as for an internal facet
    before_insertion_handle_facet_inside_conflict_zone (f);

    if(m_bad_vertices_initialized) {
      const Cell_handle& c = f.first;
      const int i = f.second;

      // for each v of f
      for (int j = 0; j < 4; j++)
        if (i != j)
          if(m_bad_vertices.erase(c->vertex(j)) > 0) {
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
            std::cerr << "m_bad_vertices.erase("
                      << c->vertex(j)->point() << ")\n";
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS
          }
    }
  }

public:
  Refine_facets_manifold_base(Tr& triangulation,
                              C3t3& c3t3,
                              const Mesh_domain& oracle,
                              const Criteria& criteria)
    : Base(triangulation,
           c3t3,
           oracle,
           criteria)
    , m_manifold_info_initialized(false)
    , m_bad_vertices_initialized(false)
    , m_with_manifold_criterion(get_with_manifold(criteria))
    , m_with_boundary(get_with_boundary(criteria))
  {
#ifdef CGAL_MESH_3_DEBUG_CONSTRUCTORS
    std::cerr << "CONS: Refine_facets_manifold_base";
    if(m_with_manifold_criterion) {
      if(m_with_boundary)
        std::cerr << " (with boundaries)\n";
      else
        std::cerr << " (without boundary)\n";
    } else {
      std::cerr << " (DEACTIVATED)\n";
    }
#endif
  }

private:
  // Initialization function
  void initialize_manifold_info() const {
#ifdef CGAL_MESH_3_VERBOSE
    std::cerr << "\nscanning edges ";
    if(m_with_boundary)
      std::cerr << "(boundaries allowed)";
    std::cerr << "...\n";
    int n = 0;
#endif
    for (typename Tr::Finite_edges_iterator
           eit = this->r_tr_.finite_edges_begin(), end = this->r_tr_.finite_edges_end();
         eit != end; ++eit)
    {
      if ( (this->r_c3t3_.face_status(*eit) == C3t3::SINGULAR) ||
           ( (!m_with_boundary) &&
             (this->r_c3t3_.face_status(*eit) == C3t3::BOUNDARY) ) )
      {
        m_bad_edges.insert(Bad_edge(edge_to_edgevv(*eit),
                                    (this->r_c3t3_.face_status(*eit) ==
                                     C3t3::SINGULAR ? 0 : 1)));
#ifdef CGAL_MESH_3_VERBOSE
        ++n;
#endif
      }
    }
    m_manifold_info_initialized = true;
#ifdef CGAL_MESH_3_VERBOSE
    std::cerr << "   -> found " << n << " bad edges\n";
#endif
  }

  void initialize_bad_vertices() const
  {
    CGAL_assertion(m_bad_vertices_initialized == false);
    CGAL_assertion(m_bad_vertices.empty());
#ifdef CGAL_MESH_3_VERBOSE
    std::cerr << "\nscanning vertices..." << std::endl;
    int n = 0;
#endif
    for (typename Tr::Finite_vertices_iterator
           vit = this->r_tr_.finite_vertices_begin(),
           end = this->r_tr_.finite_vertices_end();
         vit != end; ++vit)
    {
      if( this->r_c3t3_.face_status(vit) == C3t3::SINGULAR ) {
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
        std::cerr << "m_bad_edges.insert("
                  << vit->point() << ")\n";
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS
        m_bad_vertices.insert( vit );
#ifdef CGAL_MESH_3_VERBOSE
        ++n;
#endif
      }
    }
    m_bad_vertices_initialized = true;
#ifdef CGAL_MESH_3_VERBOSE
    std::cerr << "   -> found " << n << " bad vertices\n";
#  ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
    std::cerr << "Bad vertices queue:\n";
    BOOST_FOREACH(Vertex_handle v2, m_bad_vertices)
    {
      std::cerr << v2->point() << std::endl;
    }
    CGAL::dump_c3t3(this->r_c3t3_, "dump-at-scan-vertices");
#  endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS
#endif
  }

public:
  void scan_triangulation_impl_amendement() const {
#ifdef CGAL_MESH_3_VERBOSE
    std::cerr << "scanning edges (lazy)" << std::endl;
    std::cerr << "scanning vertices (lazy)" << std::endl;
#endif
  }

  // Tells whether there remain elements to refine
  bool no_longer_element_to_refine_impl() {
    if(Base::no_longer_element_to_refine_impl())
    {
      if(!m_with_manifold_criterion) return true;
      // Disable the manifold criterion

      if( ! m_manifold_info_initialized ) initialize_manifold_info();

      if(m_bad_edges.left.empty())
      {
        if( ! m_bad_vertices_initialized ) initialize_bad_vertices();

        return m_bad_vertices.empty();
      }
      else // m_bad_vertices is not empty
        return false;
    }
    else // Base::no_longer_element_to_refine_impl() returned false
      return false;
  }

  // Returns the next element to refine
  Facet get_next_element_impl() {

    if (!Base::no_longer_element_to_refine_impl()) {
      return Base::get_next_element_impl();
    }
    else {
      if(!m_bad_edges.left.empty()) {
        Edge first_bad_edge = edgevv_to_edge(m_bad_edges.right.begin()->second);
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
        const EdgeVV& edgevv = m_bad_edges.right.begin()->second;
        std::cerr << "Bad edge "
                  << edgevv.first->point()
                  << " - "
                  << edgevv.second->point()
                  << "\n";
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS
        return biggest_incident_facet_in_complex(first_bad_edge);
      } else {
        CGAL_assertion(!m_bad_vertices.empty());
        const Vertex_handle& v = *m_bad_vertices.begin();
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
        std::cerr << "Bad vertices queue:\n";
        BOOST_FOREACH(Vertex_handle v2, m_bad_vertices)
        {
          std::cerr << v2->point() << std::endl;
        }
        std::cerr << "Bad vertex " << v->point() << "\n";
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS
        CGAL_assertion(this->r_c3t3_.has_incident_facets_in_complex(v));
        if(this->r_c3t3_.face_status(v) != C3t3::SINGULAR) {
          dump_c3t3(this->r_c3t3_, "dump-crash");
          CGAL_error_msg("this->r_c3t3_.face_status(v) != C3t3::SINGULAR");
        }
        return biggest_incident_facet_in_complex(v);
      }
    }
  }

  void before_insertion_impl(const Facet& f, const Point& s,
                             Zone& zone) {
    if( m_manifold_info_initialized )
    {
      for (typename Zone::Facets_iterator fit =
             zone.internal_facets.begin();
           fit != zone.internal_facets.end();
           ++fit)
        before_insertion_handle_facet_inside_conflict_zone (*fit);

      for (typename Zone::Facets_iterator fit =
             zone.boundary_facets.begin(); fit !=
             zone.boundary_facets.end(); ++fit)
        before_insertion_handle_facet_on_boundary_of_conflict_zone (*fit);
    }
    Base::before_insertion_impl(f, s, zone);
  }

  void after_insertion_impl(const Vertex_handle v) {
    Base::after_insertion_impl(v);

    if( ! m_manifold_info_initialized )
      return;

    // search for incident facets around v
    typedef std::vector<Facet> Facets;
    Facets facets;
    facets.reserve(64);
    this->r_tr_.incident_facets (v, std::back_inserter(facets));

    // foreach f in star(v)
    for (typename Facets::iterator fit = facets.begin();
         fit != facets.end();
         ++fit) 
    {
      // foreach edge of *fit
      const Cell_handle cell = fit->first;
      const int i = fit->second;
      for(int j = 0; j < 3; ++j)
      {
        const int edge_index_va = this->r_tr_.vertex_triple_index(i, j);
        const int edge_index_vb = this->r_tr_.vertex_triple_index(i, (j == 2) ? 0 : (j+1));
        Edge edge(cell, edge_index_va, edge_index_vb);
        // test if edge is in Complex
        if ( this->r_c3t3_.face_status(edge) != C3t3::NOT_IN_COMPLEX ) {
          // test if edge is not regular to store it as a "bad_edge"
          // e.g. more than or equal to 3 incident facets (SINGULAR)
          // or less than or equal to 1
          // (BOUNDARY only, because ISOLATED is NA)
          // This test is not efficient because
          // edges are tried to be inserted several times
          // TODO one day: test if the edge is still singular
          if ( (this->r_c3t3_.face_status(edge) == C3t3::SINGULAR) ||
               ( (!m_with_boundary) && 
                 (this->r_c3t3_.face_status(edge) == C3t3::BOUNDARY) )
               )
          {
            m_bad_edges.insert(Bad_edge(edge_to_edgevv(edge),
                                        (this->r_c3t3_.face_status(edge) ==
                                         C3t3::SINGULAR ? 0 : 1)));
          }
          else {
            m_bad_edges.left.erase( edge_to_edgevv(edge) ); // @TODO: pourquoi?!
          }
        }
      }
    }

    if(!m_bad_vertices_initialized) return;

    // foreach v' in star of v
    std::vector<Vertex_handle> vertices;
    vertices.reserve(64);
    this->r_tr_.incident_vertices(v, std::back_inserter(vertices));

    // is_regular_or_boundary_for_vertices
    // is used here also incident edges are not known to be
    // REGULAR which may cause some singular vertices to be forgotten
    // This causes no problem because
    // those SINGULAR incident SINGULAR edges are going to be handled
    for (typename std::vector<Vertex_handle>::iterator
           vit = vertices.begin(), end = vertices.end();
         vit != end; ++vit)
    {
      if ( this->r_c3t3_.has_incident_facets_in_complex(*vit)  &&
           (this->r_c3t3_.face_status(*vit) == C3t3::SINGULAR)
           // !this->r_c3t3_.is_regular_or_boundary_for_vertices(*vit)
           )
      {
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
        std::cerr << "m_bad_vertices.insert("
                  << (*vit)->point() << ")\n";
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS
        m_bad_vertices.insert(*vit);
      }
    }

    if ( this->r_c3t3_.has_incident_facets_in_complex(v) &&
         (this->r_c3t3_.face_status(v) == C3t3::SINGULAR)
         // !this->r_c3t3_.is_regular_or_boundary_for_vertices(v)
         )
    {
#ifdef CGAL_MESHES_DEBUG_REFINEMENT_POINTS
      std::cerr << "m_bad_vertices.insert("
                << v->point() << ")\n";
#endif // CGAL_MESHES_DEBUG_REFINEMENT_POINTS
      m_bad_vertices.insert(v);
    }
  }

  /// debug info: class name
  std::string debug_info_class_name_impl() const
  {
    return "Refine_facets_manifold_base";
  }

  std::string debug_info() const
  {
    std::stringstream s;
    s << Base::debug_info();
    if(m_with_manifold_criterion) {
      s << "," << m_bad_edges.left.size()
        << "," << m_bad_vertices.size();
    }
    return s.str();
  }

  std::string debug_info_header() const
  {
    std::stringstream s;
    s << Base::debug_info_header();
    if(m_with_manifold_criterion) {
      s << ",#bad edges,#bad vertices";
    }
    return s.str();
  }
};  // end Refine_facets_manifold_base

}  // end namespace Mesh_3

}  // end namespace CGAL


#endif // CGAL_MESH_3_REFINE_FACETS_MANIFOLD_BASE_H
