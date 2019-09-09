import numpy as np
import osqp
import scipy.sparse as sparse
import scipy.linalg as sl
import rospy

class QPSolve():
    def __init__(self,dyn,clf,cbf_list,u_lim,u_cost=0.0,u_prev_cost=1.0,p1_cost=1.0e10,p2_cost=1.0e10,verbose=True):
        self.xdim = dyn.xdim
        self.udim = dyn.udim
        self.dyn = dyn
        self.clf = clf
        self.cbf_list = cbf_list
        self.u_lim = u_lim
        self.p1_cost = p1_cost
        self.p2_cost = p2_cost
        self.verbose = verbose
        self.u_cost = u_cost
        self.u_prev_cost = u_prev_cost
        self.K = 0.0
        self.ksig = 1.0
        self.max_var = 1.0
        self.mu_qp_prev = np.zeros((self.xdim/2,1))
        self.P = np.eye(self.xdim)
        self.A = np.zeros((self.xdim,self.xdim))
        self.A0 = np.block([[np.zeros((self.xdim/2,self.xdim/2)),np.eye(self.xdim/2)],[np.zeros((self.xdim/2,self.xdim/2)),np.zeros((self.xdim/2,self.xdim/2))]]) 
        self.G = np.block([[np.zeros((self.xdim/2,self.xdim/2))],[np.eye(self.xdim/2)]])
        self.res = None
        self.max_error = 1.0

    def update_ricatti(self,A):
        self.A = A
        Q = np.eye(self.xdim)
        self.P = sl.solve_continuous_are(self.A,np.zeros((self.xdim,self.xdim)),Q,np.eye(self.xdim))

    def solve(self,x,x_d,mu_d,sigDelta):
        sigDelta = sigDelta * self.ksig
        sigDelta = np.clip(sigDelta,0.0,self.max_var)
        # sigDelta = np.ones((self.xdim/2,1)) * self.max_var # for testing

        # build Q and p matrices to specify minimization expression
        Q = np.diag(np.append(np.append(np.ones(self.xdim/2)*(self.u_cost + self.u_prev_cost),self.p1_cost),self.p2_cost))
        self.Q = sparse.csc_matrix(Q)
        self.p = 2*np.append(np.append(-self.mu_qp_prev*self.u_prev_cost,0),0)

        #error dynamics for clf
        e = x[:-1,:]-x_d[:-1,:]
        e = np.clip(e,-self.max_error,self.max_error)
        eTPG = np.matmul(e.T,np.matmul(self.P,self.G))
        G_dyn = np.expand_dims(np.append(np.append(eTPG,1),0),0) #append 1 for clf < d
        Gsig = np.matmul(self.G,sigDelta)
        GssG = np.matmul(Gsig,Gsig.T)
        self.trGssGP = np.trace(np.matmul(GssG,self.P))
        h_dyn = -1 * ( -0.5*np.matmul(e.T,np.matmul(Q,e))
                    + 0.5*np.matmul(e.T,np.matmul(self.P,e)) / self.clf.epsilon
                    + 0.5*self.trGssGP)

        # build constraints for barriers
        N_cbf = len(self.cbf_list)
        G_cbf = np.zeros((N_cbf,self.xdim/2+2))
        h_cbf = np.zeros((N_cbf,1))
        for i in range(N_cbf):
            dB = self.cbf_list[i].dB(x).T
            G_cbf[i,:] = np.append(np.append(np.matmul(dB,self.G),0),1)
            trGssGd2B = np.trace(np.matmul(GssG,self.cbf_list[i].d2B(x)))
            h_cbf[i,:] = -1 * ( np.matmul(dB,np.matmul(self.A0,x[:-1,:]) + np.matmul(self.G,mu_d))
                                - self.cbf_list[i].gamma / self.cbf_list[i].B(x)
                                + 0.5*trGssGd2B)

        # build constraints for control limits
        ginv = np.linalg.inv(self.dyn.g(x))
        l_ctrl = np.matmul(ginv, mu_d - self.dyn.f(x))
        A_ctrl = ginv

        G_ctrl = np.zeros((self.udim*2,self.xdim/2+2))
        h_ctrl = np.zeros((self.udim*2,1))
        for i in range(self.udim):
            G_ctrl[i*2,:self.xdim/2] = - A_ctrl[i,:]
            h_ctrl[i*2] = - self.u_lim[i,0] + l_ctrl[i]
            G_ctrl[i*2+1,:self.xdim/2] = A_ctrl[i,:]
            h_ctrl[i*2+1] = self.u_lim[i,1] - l_ctrl[i]

        # stack into one matrix and vector
        G = np.concatenate((G_dyn,G_cbf,G_ctrl),axis=0)
        h = np.concatenate((h_dyn,h_cbf,h_ctrl),axis=0)

        self.G_csc = sparse.csc_matrix(G)
        self.h = h

        # dummy lower bound
        l = np.ones(h.shape)*np.inf * -1

        #Solve QP
        self.prob = osqp.OSQP()
        exception_called = False
        mu_bar = np.zeros((self.xdim+1))
        # try:
        self.prob.setup(P=self.Q, q=self.p, A=self.G_csc, l=l, u=self.h, verbose=self.verbose)
        self.res = self.prob.solve()
        # except:
            # exception_called = True
        # else:
        mu_bar = self.res.x
        # if exception_called or u_bar[0] is None or np.isnan(u_bar).any():
        if mu_bar[0] is None or np.isnan(mu_bar).any():
            mu_bar = np.zeros((self.xdim+1))
            self.res = None
            rospy.logwarn("QP failed!")
        

        self.mu_qp_prev = np.expand_dims(mu_bar[0:self.xdim/2],axis=0).T

        self.V =self.clf.V(x,x_d)

        if self.verbose:
            print('z_ref: ', x.T)
            print('z_des: ', x_d.T)
            print('u_lim', self.u_lim)
            print('V: ', self.V)
            print('Q:', Q)
            print('p:', np.array(self.p))
            print('G_dyn:', G_dyn)
            print('h_dyn:', h_dyn)
            print('trGssGP',self.trGssGP)
            if h_cbf.shape[0] < 10:
                print('G_cbf:', G_cbf)
                print('h_cbf:', h_cbf)
            print('G_ctrl:', G_ctrl)
            print('h_ctrl:', h_ctrl)
            print('result:', mu_bar)

        return self.mu_qp_prev
