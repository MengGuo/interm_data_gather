from P_MAS_TG.planner import ltl_planner

from sys_model import construct_sys_model

N = 1

sys_models, act_time, add_data = construct_sys_model(N)

c_buffer_size = [5,5,5,10]
buffer_size = []
for k in range(len(c_buffer_size)):
    for n in range(N):
        buffer_size.append(c_buffer_size[k])

com_rad = 5

composed_fts = compose_sys_fts(sys_models, act_time, add_data, symbols, buffer_size, com_rad)
        

        

