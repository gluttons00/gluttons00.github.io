---
title: MarkDown格式
tags: [md格式]
cover: ./images/封面-格式.png
date: 2024-05-03 22:55:51
categories: 编码格式
---

# hexo的.md文件开头

```
---
title: Hello World
tag: [格式]
第二个 "---" 不用打,按enter即可
---
```

# hexo缩进

```html
使用Html的空格符&nbsp;&emsp;控制缩进。（噩梦碎片）

&emsp;hello
hello
```

&emsp;hello

hello

# 小标题缩进

```powershell
#一级小标题
    1.空格 xxx 
    #在 1. 的行末跳格，就可以直接出现 2.
    #在 1. 的下一行输入 2.空格 结果相同
    2.空格 yyy
	#在 n. 的行末直接两次跳格就可以不出现 n+1. 的行首（即空行首）
#二级小标题
	#在 n. 的行末输入一次跳格，一次退格就可以得到二级小标题的空行首
	1.空格 xxx 
		zzz
```

1. xxx

2. yyy

   zzz

# 标题[数个"#"+空格 前置]

```
# 一级标题
## 二级标题
### 三级标题
#### 四级标题
##### 五级标题
###### 六级标题
```

# 强调 [用 "**" 或 "__" 包围]

```
**欢迎报考南京大学!** (我喜欢用这种)
__欢迎报考南京大学!__
```

**hello** 

# 斜体 [用 "*" 或 "_" 包围]

```
*欢迎大佬来浇浇我各种知识* (我喜欢用这种)
_欢迎大佬来浇浇我各种知识_
```

*hello*

# 斜体并强调

```
***hello***
```

***hello***

# 高亮[用"=="包围]

```
==hello==
```

==hello==

# 删除线 [用 "~~" 包围]

```
~~hello~~
```

~~hello~~

# 代码[用 "`" 包围]

```
`cd ..`
```

`cd ..`

# 代码块 [用"```" 并敲回车]

```
​```hello
```

```C
#include <stdio.h>
int mian() {
    print（“Hello, world!\n"）;
    return 0;
}
```

# *引用 [">" + 空格 前置]

```
> hello
>
>>hi
```

> hello
>
> >hi

# *无序列标 ["-"或"+" + 空格 前置]

```
- hello
+ hello
```

- hello

+ hello

# *上标 [用 "^" 包围]

```
2^3^
```

2^3^

# *下标 [用 "~" 包围]

```
H~2~O
```

H~2~O

# *注释 ["[^]" 后置]

```
> 今日我们相聚于此, 是为了学习 Markdown 的使用, 它的教程对于全体「观众」而言, 值得足足两个硬币的支持鼓励![^1]

[^1]: 沃兹·基·硕德 改编自「公鸡」普契涅拉.
```

> hello world![^1]

[^1]: 你好世界!

# 链接 [常用 "[ ]" + "( )" 分别包围文本与链接]

```
[hello world](https://hello world.com)
```

[hello world](https://hello world.com)

# 任务列表 ["- [ ]" + 空格 前置]

```
Todo List:
- [ ]刷B站
- [ ]写代码
- [x]起床
```



Todo List:

- [ ] 刷B站
- [ ] 写代码

- [x] 起床

# 表格 [用 "|" 绘制表格边框]

```
第一行为表头, 并由第二行分割线决定对齐方式与长度, 第三行及之后即表格数据
| 学号    | 姓名 | 年龄 |
| :------ | :--: | ---: |
| 114514  | 田所 |   24 |
| 1919810 | 浩三 |   25 |
```



| 学号    | 姓名 | 年龄 |
| :------ | :--: | ---: |
| 114514  | 田所 |   24 |
| 1919810 | 浩三 |   25 |

# 图片 [直接拖进来/复制粘贴]

![阿尔泰亚](./MarkDown格式/阿尔泰亚.png)

# 分割线 [按三个 "*" 或 "-" 或 "_" 并敲回车]

```
***/---/___
```

***

---

___

# Emoji表情 [":" 前置,后置的":"一般不需要手动输]

```
:sweat_smile:
```



:sweat_smile:

[全Emoji的网站](https://emojipedia.org/apple/)

# 公式[用"$"包围]

```
$a^n+b^n=c^n$
```

$a^n+b^n=c^n$

```
$$
%\usepackage{unicode-math}
\displaystyle \ointctrclockwise\mathcal{D}[x(t)]
\sqrt{\frac{\displaystyle3\uppi^2-\sum_{q=0}^{\infty}(z+\hat L)^{q}
\exp(\symrm{i}q^2 \hbar x)}{\displaystyle (\symsfup{Tr}\symbfcal{A})
\left(\symbf\Lambda_{j_1j_2}^{i_1i_2}\Gamma_{i_1i_2}^{j_1j_2}
\hookrightarrow\vec D\cdot \symbf P \right)}}
=\underbrace{\widetilde{\left\langle \frac{\notin \emptyset}
{\varpi\alpha_{k\uparrow}}\middle\vert
\frac{\partial_\mu T_{\mu\nu}}{2}\right\rangle}}_{\mathrm{K}_3
\mathrm{Fe}(\mathrm{CN})_6} ,\forall z \in \mathbb{R}
$$
```

$$
%\usepackage{unicode-math}
\displaystyle \ointctrclockwise\mathcal{D}[x(t)]
\sqrt{\frac{\displaystyle3\uppi^2-\sum_{q=0}^{\infty}(z+\hat L)^{q}
\exp(\symrm{i}q^2 \hbar x)}{\displaystyle (\symsfup{Tr}\symbfcal{A})
\left(\symbf\Lambda_{j_1j_2}^{i_1i_2}\Gamma_{i_1i_2}^{j_1j_2}
\hookrightarrow\vec D\cdot \symbf P \right)}}
=\underbrace{\widetilde{\left\langle \frac{\notin \emptyset}
{\varpi\alpha_{k\uparrow}}\middle\vert
\frac{\partial_\mu T_{\mu\nu}}{2}\right\rangle}}_{\mathrm{K}_3
\mathrm{Fe}(\mathrm{CN})_6} ,\forall z \in \mathbb{R}
$$



