import pdfplumber
import pandas as pd
import pickle 

content_list = []
"""
with pdfplumber.open('DSE_17_Final_Report.pdf') as pdf:
    report = pdf.pages
    print(report[45].extract_text(x_tolerance=1))
    

    for i in range(6 , len(report)):
        txt = report[i].extract_text(x_tolerance=1)
        lst = txt.replace('\n',' ').split(' ')
        content_list.append(lst)
    

print(len(content_list))
print("-------------------------------")
print(content_list[35])
#for page in report:
    
filename = 'content'
outfile = open(filename,'wb')
pickle.dump(content_list,outfile)
outfile.close()
"""


df = pd.read_excel (r'RequirementsOverview.xlsx', sheet_name='Requirement')
arr = df.to_numpy()
lst_tags = (arr.transpose()[2]).tolist()
lst_descriptions = (arr.transpose()[3]).tolist()

print(len(lst_tags),len(lst_descriptions))
unique_tags = [i for i in lst_tags if str(i) != 'nan']
unique_tags = list(dict.fromkeys(unique_tags))
unique_tags.sort()
unique_descriptions = []

for i in range(len(unique_tags)):
    for j in range(len(lst_tags)):
        if lst_tags[j] == unique_tags[i]:
            unique_descriptions.append(lst_descriptions[j])
            break
        
unique_tags = [i.replace(" ","") for i in unique_tags[:-1]]
unique_descriptions=[i.replace("\n"," ") for i in unique_descriptions[:-1]]

print(len(unique_tags),len(unique_descriptions))

for i,j in zip(unique_tags,unique_descriptions):
    print(i,'\t',j)
    print('-----------------------')

# create dictionary from requrements and save it
req_dict={ unique_tags[i] : [0,[],[],unique_descriptions[i]] for i in range(len(unique_tags)) }
print(req_dict)
filename = 'list_of_requirements'
outfile = open(filename,'wb')
pickle.dump(req_dict,outfile)
outfile.close()






