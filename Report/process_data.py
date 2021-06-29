
import pickle 

# load list of requirements 
infile = open("list_of_requirements",'rb')
requirement_dict = pickle.load(infile)
infile.close()

# load the report 
infile = open("content",'rb')
report = pickle.load(infile)

infile.close()




dict_of_chapters = {}
for i in range(2,20):
    dict_of_chapters[str(i)] = []


for page, page_num in zip(report,range(2,len(report)+2)):
    chap = page[0].strip(".")
    print(chap)
    for word in page:
        if word in requirement_dict:
            requirement_dict[word][0] +=1
            if not (page_num in requirement_dict[word][1]):
                requirement_dict[word][1].append(page_num)
            if not (chap in requirement_dict[word][2]):
                requirement_dict[word][2].append(chap)
            if not (word in dict_of_chapters[chap]):
                dict_of_chapters[chap].append(word)
            
for req in requirement_dict:         
    print(req, "\t", requirement_dict[req][0], "\t", requirement_dict[req][1])


f = open("requirement_log.txt", "w")
f.write(str(len(requirement_dict))+" requirements in total \n\n")
f.write("Tag"+'   \t'+"# in the report \t"+'pages where mentioned'+"   \t")
f.write("chapters where mentioned \n")
for req in requirement_dict:
    f.write(req+'   \t'+str(requirement_dict[req][0])+"\t")
    f.write(str(requirement_dict[req][1])+"\t")
    f.write(str(requirement_dict[req][2])+"\n")
    
f.write("\n\n\n\n")

f.write("chapter   \t list of requirements \n")
for chap in dict_of_chapters:
    f.write(chap+'   \t'+str(dict_of_chapters[chap])+"\n")

f.write("\n\n\n\n")
f.write("Tag   \t\t Description \n")
for req in requirement_dict:
    f.write(req+'   \t')
    f.write(requirement_dict[req][3]+"\n")
 
f.close()























