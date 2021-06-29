import pdfplumber
import pandas as pd
import pickle
import os 

# load list of requirements 
infile = open("list_of_requirements",'rb')
requirement_dict = pickle.load(infile)
infile.close()


Mother_string = ""
Mother_list = []
for filename in os.listdir("chapters"):
    f = open("chapters\\" + filename , "r" ,encoding="utf-8")
    Mother_string += f.read()
    f.close() 
    print(filename)

print(len(Mother_string))
Mother_list  = Mother_string.replace("\n", " ").split(" ")
print(len(Mother_list))

"""fill all information about requirements """
section = ''
for word in Mother_list:
    if "\label{sec:" in word:
        section = word.strip("\label{").strip("}")
        #print(word.strip("\label{").strip("}"))
    if word in requirement_dict:
        requirement_dict[word][0] +=1
        requirement_dict[word][1].append(section)
        #print(word)





# transform to different format




Dict = {"Tag":[],
        "Description":[],
        "Status":[],
        "Sections where expained":[]}

for req in requirement_dict:
    Dict["Tag"].append(req)
    
    Dict["Description"].append(requirement_dict[req][3])
    
    Dict["Status"].append(requirement_dict[req][4])
    
    # delete duplicates
    requirement_dict[req][1] = list(dict.fromkeys(requirement_dict[req][1]))
    
    section_string = ""
    
    for section in requirement_dict[req][1]:
        section_string+= "\\ref{"+section+"} "
        
    Dict["Sections where expained"].append(section_string)

for req in requirement_dict:
    print(req, requirement_dict[req][1],requirement_dict[req][4])

        









df = pd.DataFrame(Dict)

# Create a Pandas Excel writer using XlsxWriter as the engine.
writer = pd.ExcelWriter('demo.xlsx', engine='xlsxwriter')

# Convert the dataframe to an XlsxWriter Excel object.
df.to_excel(writer, sheet_name='Sheet1', index=False)

# Close the Pandas Excel writer and output the Excel file.
writer.save()
