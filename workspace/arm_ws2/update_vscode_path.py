import json
import os 

dir_path = os.path.dirname(os.path.realpath(__file__))
print(f'current dir: {dir_path}')

settings_file = open('.vscode/settings.json', 'r', encoding='utf-8')
settings_json = json.load(settings_file)
settings_file.close()

analysis_extra_path : list[str] = settings_json["python.analysis.extraPaths"]
new_analysis_extra_path : list[str] = []
packages_locate_path = dir_path + '/install/'

for path in analysis_extra_path:
    # print(path)
    if path.find(packages_locate_path) == -1:
        new_analysis_extra_path.append(path)
    
# print(new_analysis_extra_path)
packages_name : list[str] = next(os.walk(packages_locate_path))[1]
print(f'find packages {packages_name}')

for package_name in packages_name:
    new_analysis_extra_path.append(packages_locate_path + package_name + '/local/lib/python3.10/dist-packages')
    print(f'add new path: {new_analysis_extra_path[-1]}')

# print(new_analysis_extra_path)
settings_json["python.analysis.extraPaths"] = new_analysis_extra_path
settings_json_dump = json.dumps(settings_json, indent=4)

settings_file = open('.vscode/settings.json', 'w', encoding='utf-8')
settings_file.write(settings_json_dump)
settings_file.close()

