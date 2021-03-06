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
  void invokeLegDetectCallback( PyObject * arg );
  void invokeWakeWordCallback( PyObject * arg );
  
  PyObject * getObjectDetectCallback() const { return objectDetectCB_; }
  PyObject * getObjectTrackCallback() const { return objectTrackCB_; }
  PyObject * getLegDetectCallback() const { return legDetectCB_; }
  PyObject * getWakeWordCallback() const { return wakeWordCB_; }

  void setBaseScanCallback( PyObject * obj );
  void setTiltScanCallback( PyObject * obj );
  void setPalFaceCallback( PyObject * obj );
  void setTorsoSonarCallback( PyObject* obj );
  void setLegDetectCallback( PyObject* obj );
  void setWakeWordCallback( PyObject* obj );
  void setObjectDTCallback( PyObject * detectcb, PyObject * trackcb );

  void invokeObjectDetectionCallback( PyObject * arg );
  void invokeObjectTrackingCallback( PyObject * arg );

private:
  static PyREEMModule * s_pyREEMModule;
  
  PyObject * baseScanCB_;
  PyObject * tiltScanCB_;
  PyObject * palFaceCB_;
  PyObject * torsoSonarCB_;
  PyObject * objectDetectCB_;
  PyObject * objectTrackCB_;
  PyObject * legDetectCB_;
  PyObject * wakeWordCB_;

  PyREEMModule();
  PyObject * createPyModule();
};

} // namespace pyride

#endif // PY_REEM_MODULE_H
