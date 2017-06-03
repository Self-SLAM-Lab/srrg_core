#include "cloud_3d.h"

#ifdef _GO_PARALLEL_
#include <omp.h>
#endif

#include <GL/gl.h>
#include <string>
#include <stdexcept>

// #define _NAN_CHECK_

namespace srrg_core {

  using namespace std;
  using namespace srrg_boss;
  using namespace srrg_core;

  void Cloud3D::add(const Cloud3D& other) {
    if (&other == this)
      return;
    size_t k = size();
    resize(k + other.size());
#ifdef _GO_PARALLEL_
#pragma omp parallel for
#endif
    for (size_t i = 0; i < other.size(); i++) {
      at(k + i) = other.at(i);
    }
  }


  void Cloud3D::draw(int name) const {
    if (name > -1)
      glPushName(name);

    glBegin(GL_POINTS);
    for (size_t i = 0; i<size(); i++){
      const RichPoint3D& rp = at(i);
      const Eigen::Vector3f& p = rp.point();
      const Eigen::Vector3f& n = rp.normal();
      if (n.squaredNorm()) {
	const Eigen::Vector3f& rgb = rp.rgb();
	if (rgb.squaredNorm()){
	  glPushAttrib(GL_COLOR);
	  glColor3f(rgb.x(), rgb.y(), rgb.z());
	  glNormal3f(n.x(), n.y(), n.z());
	  glVertex3f(p.x(), p.y(), p.z());
	  glPopAttrib();
	} else {
	  glNormal3f(n.x(), n.y(), n.z());
	  glVertex3f(p.x(), p.y(), p.z());
	}
      } else {
	Eigen::Vector3f n = -rp.normal().normalized();
	glNormal3f(n.x(), n.y(), n.z());
	glVertex3f(p.x(), p.y(), p.z());
      }
    }
    glEnd();
    if (name > -1)
      glPopName();
  }

  void Cloud3D::transformInPlace(const Eigen::Isometry3f& T) {
    Eigen::Matrix3f R = T.linear();
#ifdef _GO_PARALLEL_
#pragma omp parallel for
#endif
    for (size_t i = 0; i < size(); i++) {
      if (at(i).accumulator() <= 0) {
	cerr << "size: " << size() << endl;
	cerr << "index: " << i << endl;
	cerr << "p: " << at(i).point().transpose() << endl;
	cerr << "n: " << at(i).normal().transpose() << endl;
	cerr << "cvi: " << at(i).accumulator() << endl;
	throw std::runtime_error("cum numm");
      }
      
      at(i).transformInPlace(T);
    }
  }

  void Cloud3D::transform(Cloud3D& other, const Eigen::Isometry3f& T) const {
    other.resize(size());
    Eigen::Matrix3f R = T.linear();
#ifdef _GO_PARALLEL_
#pragma omp parallel for
#endif
    for (size_t i = 0; i < size(); i++) {
      other[i] = at(i).transform(T);
    }
  }


  struct IndexTriplet {
    int x, y, z, index;
    IndexTriplet() {
      x = y = z = 0;
      index = -1;
    }

    IndexTriplet(const Eigen::Vector3f& v, int idx, float ires) {
      x = (int) (ires * v.x());
      y = (int) (ires * v.y());
      z = (int) (ires * v.z());
      index = idx;
    }
    bool operator < (const IndexTriplet& o) const {
      if (z < o.z)
	return true;
      if (z > o.z)
	return false;
      if (x < o.x)
	return true;
      if (x > o.x)
	return false;
      if (y < o.y)
	return true;
      if (y > o.y)
	return false;
      if (index < o.index)
	return true;
      return false;
    }

    bool sameCell( const IndexTriplet& o) const {
      return x == o.x && y == o.y && z == o.z;
    }
  };

  //! clips to a maxRange around a pose
  void Cloud3D::clip(float range, const Eigen::Isometry3f& pose) {
    Eigen::Isometry3f T = pose.inverse();
    range *= range;
    int k = 0;
    for (size_t i = 0; i < size(); i++) {
      const RichPoint3D& p = at(i);
      Eigen::Vector3f other_p = T * p.point();
      if (other_p.squaredNorm() < range) {
	at(k) = p;
	k++;
      }
    }
    resize(k);
  }


  void Cloud3D::voxelize(float res) {
    float ires = 1. / res;
    std::vector<IndexTriplet> voxels(size());
    for (size_t i= 0; i < size(); ++i) {
      const RichPoint3D& p=(*this)[i];
      voxels[i] = IndexTriplet(p.point(), i , ires);
    }
    Cloud3D sparse_model;
    sparse_model.resize(size());
    std::sort(voxels.begin(), voxels.end());
    int k = -1;
    for (size_t i = 0; i < voxels.size(); i++) {
      IndexTriplet& triplet = voxels[i];
      int idx = triplet.index;
      if (k >= 0  && voxels[i].sameCell(voxels[i - 1])) {
	sparse_model[k] += at(idx);
      } else {
	k++;
	sparse_model[k] = at(idx);
      }
      //cerr << voxels[i].x << " " << voxels[i].y << " " << voxels[i].x <<  endl;
    }
    for (int i = 0; i < k; i++) {
      RichPoint3D& p=sparse_model[i];
      if (p.accumulator() <= 0)
	throw std::runtime_error("null accumulator");
      p.normalize();
      (*this)[i]=p;
    }
    resize(k);
  }

  template <class T>
  void writeDatum(ostream& os, const T& d) {
    const char * dp = reinterpret_cast<const char*>(&d);
    os.write(dp, sizeof(T));
  }

  void Cloud3D::write(ostream& os) const {
    size_t s=size();
    writeDatum(os, s);
    for(size_t i=0; i<s; i++) {
      const RichPoint3D& p=at(i);
      writeDatum(os, p.point().x());
      writeDatum(os, p.point().y());
      writeDatum(os, p.point().z());
      writeDatum(os, p.normal().x());
      writeDatum(os, p.normal().y());
      writeDatum(os, p.normal().z());
      writeDatum(os, p.rgb().x());
      writeDatum(os, p.rgb().y());
      writeDatum(os, p.rgb().z());
      float acc = p.accumulator();
      writeDatum(os, acc);
    } 
  }

  template <class T>
  void readDatum(istream& is, T& d) {
    char * dp = reinterpret_cast<char*>(&d);
    is.read(dp, sizeof(T));
  }


  void Cloud3D::computeBoundingBox(Eigen::Vector3f& lower, Eigen::Vector3f& higher) const {
    const float low=-std::numeric_limits<float>::max();
    const float up=std::numeric_limits<float>::max();
    lower=Eigen::Vector3f(up, up, up);
    higher=Eigen::Vector3f(low, low, low);
    for(size_t i=0; i<size(); i++) {
      const RichPoint3D& p=at(i);
      if (lower.x()>p.point().x())
	lower.x()=p.point().x();
      if (lower.y()>p.point().y())
	lower.y()=p.point().y();
      if (lower.z()>p.point().z())
	lower.z()=p.point().z();
      if (higher.x()<p.point().x())
	higher.x()=p.point().x();
      if (higher.y()<p.point().y())
	higher.y()=p.point().y();
      if (higher.z()<p.point().z())
	higher.z()=p.point().z();
    }
  }

  bool Cloud3D::read(istream& is) {
    clear();
    size_t s;
    readDatum(is, s);
    resize(s);
    for(size_t i=0; i<s; i++) {
      RichPoint3D p=at(i);
      float x,y,z, nx, ny, nz, cx, cy,cz, a;
      readDatum(is, x);
      readDatum(is, y);
      readDatum(is, z);
      readDatum(is, nx);
      readDatum(is, ny);
      readDatum(is, nz);
      readDatum(is, cx);
      readDatum(is, cy);
      readDatum(is, cz);
      readDatum(is, a);
      at(i) = RichPoint3D(Eigen::Vector3f(x,y,z), Eigen::Vector3f(nx, ny, nz), a, Eigen::Vector3f(cx, cy, cz));
    } 
    return true;
  }

  BOSS_REGISTER_BLOB(Cloud3D);

}

