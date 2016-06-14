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
  
  void setBaseScanCallback( PyObject * obj );
  void setTiltScanCallback( PyObject * obj );
  void setPalFaceCallback( PyObject * obj );

#ifdef WITH_REEMHT
  void setObjectDTCallback( PyObject * detectcb, PyObject * trackcb );
  
  void invokeObjectDetectionCallback( PyObject * arg );
  void invokeObjectTrackingCallback( PyObject * arg );
#endif

#ifdef WITH_RHYTH_DMP
  void setTrajectoryInputCallback( PyObject * inputcb );
  void invokeTrajectoryInputCallback( PyObject * arg );
#endif

private:
  static PyREEMModule * s_pyREEMModule;
  
  PyObject * baseScanCB_;
  PyObject * tiltScanCB_;
  PyObject * palFaceCB_;
  
#ifdef WITH_REEMHT
  PyObject * objectDetectCB_;
  PyObject * objectTrackCB_;
#endif

#ifdef WITH_RHYTH_DMP
  PyObject * trajInputCB_;
#endif

  PyREEMModule();
  PyObject * createPyModule();
};

} // namespace pyride

#endif // PY_REEM_MODULE_H
