#include <stdio.h>
#include <stdlib.h>

void sampleCollect(int n,float array[2][5],float total_prob[5]);

void main()
{
    int n;        //输入样本的个数
    float good_prob[2][5],notgood_prob[2][5];//用good_prob的第三行把total-prob传回来
    int i,j;
    float info[4],aspect[4];//info用户输入的特征列表,aspect存放需要计算的条件概率
    float p_marry=1,p_notmarry=1;
    float cons=1,total_prob[5];//帅,性格好,高,上进的总概率


    printf("请输入样本集\n");

    printf("输入样本的条数n:");
    scanf("%d",&n);

    sampleCollect(n,good_prob,total_prob);
    //求notgood_prob矩阵的值
    for(i=0; i<2; i++)
        for(j=0; j<5; j++)
            notgood_prob[i][j]=1-good_prob[i][j];

    //打印good_prob
    printf("good_prob array:\n");
    for(i=0; i<2; i++)
        for(j=0; j<5; j++)
            printf("%f\t",good_prob[i][j]);
    //打印notgood_prob
    printf("notgood_prob array:\n");
    for(i=0; i<2; i++)
        for(j=0; j<5; j++)
            printf("%f\t",notgood_prob[i][j]);
    //打印total_prob
    printf("total_prob list:\n");
    for(i=0; i<5; i++)
        printf("%f\t",total_prob[i]);

    while(info[0]!=6)
    {
        //测试
        printf("请输入特征:(按6退出)\n");
        printf("他很帅输入1,否则输入0:");
        scanf("%f",&info[0]);

        printf("性格好输入1,否则输入0:");
        scanf("%f",&info[1]);

        printf("他长得高输入1,矮输入0:");
        scanf("%f",&info[2]);

        printf("他很上进输入1,否则输入0:");
        scanf("%f",&info[3]);

        //先算嫁的概率,对每一个特性进行判断
        cons=total_prob[4];
        for(j=0; j<4; j++)
        {
            if(1==info[j])
            {
                aspect[j]=good_prob[0][j];//如果是好品质就从goog_prob列表里取数
                cons/=total_prob[j];
            }
            else
            {
                aspect[j]=notgood_prob[0][j];
                cons/=(1-total_prob[j]);
            }
        }
        p_marry=(aspect[0])*(aspect[1])*(aspect[2])*(aspect[3])*cons;
        printf("\n嫁的概率是:%f\n",p_marry);

        //再算不嫁的概率:
        cons=1-total_prob[4];
        for(j=0; j<4; j++)
        {
            if(1==info[j])
            {
                aspect[j]=good_prob[1][j];//不嫁里面特征好的,从goog_prob列表里取数
                cons/=total_prob[j];
            }
            else
            {
                aspect[j]=notgood_prob[1][j];//不嫁里特征不好的
                cons/=(1-total_prob[j]);
            }
        }
        p_notmarry=(aspect[0])*(aspect[1])*(aspect[2])*(aspect[3])*cons;
        printf("不嫁的概率是:%f\n",p_notmarry);

        if(p_marry>p_notmarry)
            printf("这个男人可以嫁\n");
        else
            printf("这个男人不能嫁\n");
    }
}

void sampleCollect(int n,float array[2][5],float total_prob[5])
{
    int i,j,k=0;
    float features[n][5];
    float handsome=0,kind=0,tall=0,posit=0,marry=0;
    float good_m=0,good_nm=0;

    //记录样本
    for(i=0; i<n; i++)
    {
        printf("输入4个特征:帅,性格好,长得高,上进,输入1或0:");
        scanf("%f %f %f %f %f",&features[i][0],&features[i][1],&features[i][2],&features[i][3],&features[i][4]);
        if(features[i][0]==1)
        {
            handsome++;
            total_prob[0]=handsome/n;
        }
        if(1==features[i][1])
        {
            kind++;
            total_prob[1]=kind/n;
        }
        if(1==features[i][2])
        {
            tall++;
            total_prob[2]=tall/n;
        }
        if(1==features[i][3])
        {
            posit++;
            total_prob[3]=posit/n;
        }
        if(1==features[i][4])
        {
            marry++;
            total_prob[4]=marry/n;
        }
    }

    //计算good概率
    for(j=0; j<4; j++)
    {
        good_m=0;
        good_nm=0;
        for(i=0; i<n; i++)
        {
            if(features[i][j]==1)//好
            {
                if(features[i][4]==1)//好_嫁
                    good_m++;
                else
                    good_nm++;//好_不嫁
            }
        }
        array[0][j]= good_m/marry;
        //printf("%f\n",array[0][j]);
        array[1][j]=good_nm/(n-marry);
        //printf("%f\n",array[1][j]);
    }
    array[0][4]=marry/n;
    array[1][4]=(n-marry)/n;
}


