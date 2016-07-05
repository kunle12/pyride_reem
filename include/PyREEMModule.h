/*
 *  PyREEMModule.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 18/06/12.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#ifndef PY_REEM_MODULE_H
#define PY_REEM_MODULE_H

#include <PyModuleStub.h>

namespace pyride {

class PyREEMModule : public PyModuleExtension
{
public:
  static PyREEMModule * instance();
  virtual ~PyREEMModule();
  
  void invokeBaseScanCallback( PyObject * arg );
  void invokeTiltScanCallback( PyObject * arg );
  void invokePalFaceCallback( PyObject * arg );
  void invokeTorsoSonarCallback( PyObject * arg );
  
  void setBaseScanCallback( PyObject * obj );
  void setTiltScanCallback( PyObject * obj );
  void setPalFaceCallback( PyObject * obj );
  void setTorsoSonarCallback( PyObject* obj );

private:
  static PyREEMModule * s_pyREEMModule;
  
  PyObject * baseScanCB_;
  PyObject * tiltScanCB_;
  PyObject * palFaceCB_;
  PyObject * torsoSonarCB_;

  PyREEMModule();
  PyObject * createPyModule();
};

} // namespace pyride

#endif // PY_REEM_MODULE_H
