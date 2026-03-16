class KalmanFilter1D:
    def __init__(self,x0,p0,q,r):
        self.x=x0;self.p=p0;self.q=q;self.r=r
    def predict(self,u=0):
        self.x=self.x+u; self.p=self.p+self.q
    def update(self,z):
        k=self.p/(self.p+self.r)
        self.x=self.x+k*(z-self.x)
        self.p=(1-k)*self.p
        return self.x
class KalmanFilter:
    def __init__(self,dim):
        self.dim=dim
        self.x=[0]*dim;self.P=[[1 if i==j else 0 for j in range(dim)] for i in range(dim)]
    def predict(self,F,Q):
        # x = F*x, P = F*P*F^T + Q
        self.x=[sum(F[i][j]*self.x[j] for j in range(self.dim)) for i in range(self.dim)]
        new_P=[[sum(F[i][k]*self.P[k][l]*F[j][l] for k in range(self.dim) for l in range(self.dim))+Q[i][j] for j in range(self.dim)] for i in range(self.dim)]
        self.P=new_P
if __name__=="__main__":
    import random; random.seed(42)
    kf=KalmanFilter1D(x0=0,p0=1,q=0.1,r=1)
    true_pos=0; estimates=[]
    for i in range(50):
        true_pos+=1
        measurement=true_pos+random.gauss(0,1)
        kf.predict(u=1)
        est=kf.update(measurement)
        estimates.append(est)
    error=abs(estimates[-1]-50)
    print(f"True: 50, Estimated: {estimates[-1]:.2f}, Error: {error:.2f}")
    assert error<2
    print("All tests passed!")
