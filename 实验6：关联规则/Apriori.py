#!/usr/bin/env python
# coding: utf-8

# In[2]:


import copy
Dataset=[{'面包','牛奶','茶'}, #集合里元素不重复，所以不能包含２个相同商品
         {'面包','尿布','啤酒','茶'},
         {'牛奶','尿布','啤酒'},
         {'面包','牛奶','尿布','茶'},
         {'面包','尿布','牛奶'},
         {'面包','牛奶','啤酒','尿布','茶'},
         {'啤酒','牛奶','茶'},
         {'面包','茶'},
         {'面包','尿布','牛奶','啤酒','茶'},
         {'面包','牛奶'}]


# In[6]:


#统计支持度个数
goods_dict={} #商品字典：5种商品
for set_origin in Dataset: #统计每种商品的支持度
    for thing in set_origin:
        if thing not in goods_dict:
            goods_dict[thing]=0
        goods_dict[thing]+=1
#print(goods_dict)

while(len(goods_dict)>1):
    #剪枝
    frequent_list=[]
    for item in goods_dict.items():#每一个item是一个元组
        if (item[1]>0.35*len(goods_dict)):
            frequent_list.append(item[0]) #大于支持度的项集列表
    #print(frequent_list)

    #连接
    new_compose_list=[] #商品新组合
    copy_frequent_list=copy.copy(frequent_list) #拷贝一份以便删除
    for element in frequent_list: #频繁项集里的每一项集与其他的项集组合，当他们两个补集等于2时可以连接
        copy_frequent_list.remove(element)#和剩下的搭配
        for goods in copy_frequent_list:
            if len(set({goods})^set({element}))==2:
                union_set=set({goods})^set({element})
                new_compose_list.append(union_set)#因元组是有序的,在后面不能完全比对出来,所以还是用集合吧
    print(new_compose_list)
    #print(Dataset[0].issubset(Dataset[0]))

    #统计支持度个数
    new_goods_dict={}
    for new_set in new_compose_list:
        for original_set in Dataset:
            if new_set.issubset(original_set):
                if tuple(new_set) not in new_goods_dict.keys():
                    new_goods_dict[tuple(new_set)]=0
                new_goods_dict[tuple(new_set)]+=1
    #print(new_goods_dict)
    goods_dict=new_goods_dict #商品字典更新
else:
    print(goods_dict)


# In[ ]:




