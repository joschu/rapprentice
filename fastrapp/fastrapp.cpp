#include <boost/python.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include "numpy_utils.hpp"
#include <Eigen/Core>
#include <iostream>
using namespace Eigen;
using namespace std;

namespace py = boost::python;

bool gInteractive = false;
py::object openravepy;

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

BOOST_PYTHON_MODULE(fastrapp) {

  np_mod = py::import("numpy");

  py::def("resample", &pyResample, (py::arg("x"), py::arg("t"), py::arg("max_err"), py::arg("max_dx")=INFINITY, py::arg("max_dt")=INFINITY));

}
