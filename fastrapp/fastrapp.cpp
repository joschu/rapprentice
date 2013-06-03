#include "numpy_utils.hpp"
#include <boost/foreach.hpp>
#include <boost/python.hpp>
#include <Eigen/Core>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <iostream>
#include <iostream>
#include <vector>


using namespace Eigen;
using namespace std;
namespace py = boost::python;


/////////////////




// http://en.wikipedia.org/wiki/Axis_angle#Log_map_from_SO.283.29_to_so.283.29
Vector3d LogMap(const Matrix3d& m) {
  double cosarg = (m.trace() - 1)/2;
  cosarg = fmin(cosarg, 1);
  cosarg = fmax(cosarg, -1);
  double theta = acos( cosarg );
  if (theta==0) return Vector3d::Zero();
  else return theta*(1/(2*sin(theta))) * Vector3d(m(2,1) - m(1,2), m(0,2)-m(2,0), m(1,0)-m(0,1));
}

double RotReg(const Matrix3d& b, const Vector3d& rot_coeffs, double scale_coeff) {
  // regularize rotation using polar decomposition
  JacobiSVD<Matrix3d> svd(b.transpose(), ComputeFullU | ComputeFullV);
  Vector3d s = svd.singularValues();
  if (b.determinant() <= 0) return INFINITY;
  return LogMap(svd.matrixU() * svd.matrixV().transpose()).cwiseAbs().dot(rot_coeffs) + s.array().log().square().sum()*scale_coeff;
}

Matrix3d RotRegGrad(const Matrix3d& b, const Vector3d& rot_coeffs, double scale_coeff) {
  Matrix3d out;
  double y0 = RotReg(b, rot_coeffs, scale_coeff);
  Matrix3d xpert = b;
  double epsilon = 1e-5;
  for (int i=0; i < 3; ++i) {
    for (int j=0; j < 3; ++j) {
      xpert(i,j) = b(i,j) + epsilon;
      out(i,j) = (RotReg(xpert, rot_coeffs, scale_coeff) - y0)/epsilon;
      xpert(i,j) = b(i,j);
    }
  }
  return out;
}

Vector3d gRotCoeffs;
double gScaleCoeff;

void PySetCoeffs(py::object rot_coeffs, py::object scale_coeff) {
  gRotCoeffs = Vector3d(py::extract<double>(rot_coeffs[0]), py::extract<double>(rot_coeffs[1]), py::extract<double>(rot_coeffs[2]));
  gScaleCoeff = py::extract<double>(scale_coeff);
}


double PyRotReg(const py::object& m ){
  const double* data = getPointer<double>(m);
  return RotReg( Map< const Matrix<double,3,3,RowMajor> >(data), gRotCoeffs, gScaleCoeff);
}

py::object PyRotRegGrad(const py::object& m) {
  static py::object np_mod = py::import("numpy");
  py::object out = np_mod.attr("empty")(py::make_tuple(3,3));
  const double* data = getPointer<double>(m);
  Matrix<double,3,3,RowMajor> g = RotRegGrad( Map< const Matrix<double,3,3,RowMajor> >(data), gRotCoeffs, gScaleCoeff);
  memcpy(getPointer<double>(out), g.data(), sizeof(double)*9);
  return out;
}



////////////////

void Interp(float x0, float x1, const VectorXf& y0, const VectorXf& y1, const VectorXf& newxs, MatrixXf& newys) {
  float dx = x1 - x0;
  for (int i=0; i < newxs.size(); ++i) {
    newys.row(i) = ((x1 - newxs[i])/dx) * y0 + ((newxs[i] - x0)/dx) * y1;
  }
}

vector<int> Resample(const MatrixXf& x, const VectorXf& _t, float max_err, float max_dx, float max_dt) {
  int N = x.rows(); // number of timesteps
  VectorXf t;
  if (_t.size() == 0){
    t.resize(N);
    for (int i=0; i < N; ++i) t[i] = i;
  }
  else t=_t;
  VectorXi cost(N); // shortest path cost
  VectorXi pred(N); // shortest path predecessor

  MatrixXf q(20, x.cols()); // scratch space for interpolation
  q.setConstant(-666);

  const int NOPRED = -666;
  const int BIGINT = 999999;

  pred.setConstant(NOPRED);
  cost.setConstant(BIGINT);
  cost(0) = 0;
  pred(0) = NOPRED;
  for (int iSrc = 0; iSrc < N; ++iSrc) {
    for (int iTarg = iSrc+1; iTarg < N; ++iTarg) {
      float dx = (x.row(iTarg) - x.row(iSrc)).maxCoeff();
      float dt = t(iTarg) - t(iSrc);
      int seglen = iTarg - iSrc + 1;
      if (q.rows() < seglen) q.resize(2*q.rows(), q.cols());
      Interp(t(iSrc), t(iTarg), x.row(iSrc), x.row(iTarg), t.middleRows(iSrc, seglen), q);
      float err = (q.block(0,0,seglen, x.cols()) - x.middleRows(iSrc, seglen)).cwiseAbs().maxCoeff();
      if ((dx <= max_dx) && (dt <= max_dt || iTarg == iSrc+1) && (err <= max_err)) {
        int newcost = cost(iSrc) + 1;
        if (newcost < cost(iTarg)) {
          cost(iTarg) = newcost;
          pred(iTarg) = iSrc;
        }
      }
      else break;
    }
  }

  int i=N-1;
  vector<int> revpath;
  while (i > 0) {
    revpath.push_back(i);
    i = pred(i);
  }
  revpath.push_back(0);
  std::reverse(revpath.begin(), revpath.end());
  return revpath;
}

py::object pyResample(py::object x, py::object t, float max_err, float max_dx, float max_dt) {
  x = np_mod.attr("array")(x, "float32");
  int xdim0 = py::extract<int>(x.attr("shape")[0]);
  int xdim1 = py::extract<int>(x.attr("shape")[1]);
  float* xdata = getPointer<float>(x);
  t = np_mod.attr("array")(t, "float32");
  int tdim0 = py::extract<int>(t.attr("__len__")());
  float* tdata = getPointer<float>(t);
  vector<int> inds = Resample(Map<MatrixXf>(xdata, xdim0, xdim1), Map<VectorXf>(tdata, tdim0), max_err, max_dx, max_dt);
  return toNdarray1<int>(inds.data(), inds.size());
}
//BOOST_PYTHON_FUNCTION_OVERLOADS(resample_overloads, pyResample, 3, 5);

///////////////////


BOOST_PYTHON_MODULE(fastrapp) {

  np_mod = py::import("numpy");

  py::def("resample", &pyResample, (py::arg("x"), py::arg("t"), py::arg("max_err"),   py::arg("max_dx")=INFINITY, py::arg("max_dt")=INFINITY));
  
  py::def("set_coeffs", &PySetCoeffs, (py::arg("rot_coeffs"), py::arg("scale_coeff")));
  py::def("rot_reg", &PyRotReg, (py::arg("B")));
  py::def("rot_reg_grad", &PyRotRegGrad, (py::arg("B")));
  

}
