

txt1 = ["a", "b", "c"]
txt2 = True
txt3 = "False"


txt1.append(txt2)
print(txt1)
txt1.append(str(txt2))
print(txt1)
txt1.append(txt3)
print(txt1)

# txt1.extend(txt2)
# print(txt1)
txt1.extend(str(txt2))
print(txt1)
txt1.extend(txt3)
print(txt1)