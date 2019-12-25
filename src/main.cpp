#include <iostream>
#include <SerialManipulator.h>
#include <Controller.h>
int main()
{
    SerialManipulator cManipulator;
    HYUControl::Controller Control(&cManipulator);

    double q[15], qdot[15];
    double qd[15], qd_dot[15], qd_ddot[15];
    double toq[15];
    double dt;
    VectorXd q_(15), qdot_(15), qd_(15), qddot_(15), qdddot_(15);
    MatrixXd pInvJac, AJac;

    for(int i=0; i<4; i++)
    {
        cManipulator.pKin->PrepareJacobian(q);
        cManipulator.pDyn->PrepareDynamics(q, qdot);

        cManipulator.pKin->GetpinvJacobian(pInvJac);
        cManipulator.pKin->GetAnalyticJacobian(AJac);

        Control.InvDynController(q_, qdot_, qd_, qddot_, qdddot_, toq, dt);

    }

    return 0;
}