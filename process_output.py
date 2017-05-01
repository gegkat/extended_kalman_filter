import numpy as np
import matplotlib.pyplot as plt
import pylab

def plot_output():
  f = open('build/output.txt')
  # est_px est_py est_vx est_vy meas_px meas_py gt_px gt_py gt_vx gt_vy
  data = []
  for line in f:
    words = line.split('\t')
    row = []
    for word in words:
      row.append(float(word)) 
    data.append(row)

  m = np.matrix(data)
  px = m[:,0]
  py = m[:,1]
  vx = m[:,2]
  vy = m[:,3]
  measx = m[:,4]
  measy = m[:,5]
  gtpx = m[:,6]
  gtpy = m[:,7]
  gtvx = m[:,8]
  gtvy = m[:,9]
  t = m[:,10]
  #t = (t-t[0])*1e-6
  t = range(px.shape[0])
 
  plt.subplot(2,2,1)
  plt.plot(t, px)
  plt.plot(t, gtpx)
  plt.plot(t, measx)
  plt.subplot(2,2,2)
  plt.plot(t, py)
  plt.plot(t, gtpy)
  plt.plot(t, measy)
  plt.subplot(2,2,3)
  plt.plot(t, vx)
  plt.plot(t, gtvx)
  plt.subplot(2,2,4)
  # plt.plot(t, vy)
  # plt.plot(t, gtvy)
  plt.plot(px, py)
  plt.plot(gtpx, gtpy)
  plt.show()

# def plot_in():
#L(for laser) meas_px meas_py timestamp gt_px gt_py gt_vx gt_vy
#R(for radar) meas_rho meas_phi meas_rho_dot timestamp gt_px gt_py gt_vx gt_vy

  # f = open('./data/obj_pose-laser-radar-synthetic-input.txt', 'r')
  # data = []
  # for line in f:
  #   words = line.split(line)
  #   row = []
  #   for word in words:


if __name__ == '__main__':
  plot_output()
