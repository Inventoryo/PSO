import os
import shutil 
if __name__ == '__main__':
    folder_path = 'D:/Workspace/PSO/'
    main_exe = folder_path + 'mingw_build/main_test.exe'
    for i in range(1):
        for j in range(30):
            config_path = folder_path + 'config/config' + str(i) + '.xml'
            src_path = folder_path + 'temp/'
            if not os.path.exists(src_path):
                os.makedirs(src_path)
            para = "%s %s" % (main_exe, config_path)
            cur_commond = main_exe + ' ' + config_path
            f = os.system(para) 
            output_path = folder_path + 'eta_20230226/' + str(i) + '/' + str(j)+ '/'
            src_path = folder_path + 'temp/'
            shutil.move(src=src_path, dst=output_path)

