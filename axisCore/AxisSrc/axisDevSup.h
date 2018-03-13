/*****************************************************************************
  Calls to device support
  Wrappers that call device support.
*****************************************************************************/
#ifndef INC_axisDevSup_H
#define INC_axisDevSup_H

#ifdef __cplusplus
extern "C" {
#endif
/*******************************************************************************
Support for tracking the progress of motor from one invocation of 'process()'
to the next.  The field 'pmr->mip' stores the motion in progress using these
fields.  ('pmr' is a pointer to axisRecord.)
*******************************************************************************/
#define MIP_DONE        0x0000  /* No motion is in progress. */
#define MIP_JOGF        0x0001  /* A jog-forward command is in progress. */
#define MIP_JOGR        0x0002  /* A jog-reverse command is in progress. */
#define MIP_JOG_BL1     0x0004  /* Done jogging; 1st phase take out backlash. */
#define MIP_JOG         (MIP_JOGF | MIP_JOGR | MIP_JOG_BL1 | MIP_JOG_BL2)
#define MIP_HOMF        0x0008  /* A home-forward command is in progress. */
#define MIP_HOMR        0x0010  /* A home-reverse command is in progress. */
#define MIP_HOME        (MIP_HOMF | MIP_HOMR)
#define MIP_MOVE        0x0020  /* A move not resulting from Jog* or Hom*. */
#define MIP_RETRY       0x0040  /* A retry is in progress. */
#define MIP_LOAD_P      0x0080  /* A load-position command is in progress. */
#define MIP_MOVE_BL     0x0100  /* Done moving; now take out backlash. */
#define MIP_STOP        0x0200  /* We're trying to stop.  When combined with */
/*                                 MIP_JOG* or MIP_HOM*, the jog or home     */
/*                                 command is performed after motor stops    */
#define MIP_DELAY_REQ   0x0400  /* We set the delay watchdog */
#define MIP_DELAY_ACK   0x0800  /* Delay watchdog is calling us back */
#define MIP_DELAY       (MIP_DELAY_REQ | MIP_DELAY_ACK) /* Waiting for readback
                                                         * to settle */
#define MIP_JOG_REQ     0x1000  /* Jog Request. */
#define MIP_JOG_STOP    0x2000  /* Stop jogging. */
#define MIP_JOG_BL2     0x4000  /* 2nd phase take out backlash. */
#define MIP_EXTERNAL    0x8000  /* Move started by external source */


  void devSupStop(axisRecord *pmr);
  void devSupLoadPos(axisRecord *pmr, double newpos);
  RTN_STATUS devSupGetInfo(axisRecord *pmr);
  RTN_STATUS devSupUpdateLimitFromDial(axisRecord *pmr, motor_cmnd command,
                                       double dialValue);
  void devSupMoveAbsRaw(axisRecord *pmr, double vel, double vbase,
                        double acc, double pos);
  void devSupMoveRelRaw(axisRecord *pmr, double vel, double vbase,
                        double acc, double relpos);
  void devSupJogDial(axisRecord *pmr, double jogv, double jacc);
  void devSupUpdateJogRaw(axisRecord *pmr, double jogv, double jacc);
  void devSupCNEN(axisRecord *pmr, double cnen);
  int  devSupSetPID(axisRecord *pmr, motor_cmnd command, double *pcoeff);
  void devSupSetEncRatio(axisRecord *pmr, double ep_mp[2]);
  void setCDIRfromRawMove(axisRecord *pmr, int directionRaw);
  void setCDIRfromDialMove(axisRecord *pmr, int directionDial);
  void doHomeSetcdir(axisRecord *pmr);

#ifdef __cplusplus
}
#endif
#endif /* INC_axisDevSup_H */
