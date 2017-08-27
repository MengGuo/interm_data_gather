from P_MAS_TG.planner import ltl_planner

from sys_model import construct_sys_model, construct_sys_model_small, compose_sys_ltl, compose_sys_fts

import time


t0 = time.time()
N = 1

#sys_models, add_data, symbols = construct_sys_model(N)
sys_models, add_data, symbols = construct_sys_model_small(N)
print 'System models built in %.2f' %(time.time()-t0)

c_buffer_size = [5,5,5,1]
buffer_size = []
for k in range(len(c_buffer_size)):
    for n in range(N):
        buffer_size.append(c_buffer_size[k])

com_rad = 1

t0 = time.time()
composed_fts = compose_sys_fts(sys_models, add_data, symbols, buffer_size, com_rad)
print 'Composed fts done in %.2f' %(time.time()-t0)

# t0 = time.time()
# composed_task = compose_sys_ltl(sys_models)
# print 'Composed task done in %.2f' %(time.time()-t0)

# t0 = time.time()
# composed_planner = ltl_planner(composed_fts, composed_task, None)
# print 'Composed planner initialized  in %.2f' %(time.time()-t0)

# t0 = time.time()
# composed_planner.optimal(style='ready')
# print 'Optimal plan found in %.2f' %(time.time()-t0)

