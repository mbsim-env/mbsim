%%
%% This is file `AMpaper.cls',
%% generated with the docstrip utility.
%%
%% The original source files were:
%%
%% AMclasses.dtx  (with options: `class,AMpaper')
%% 
%% This is a generated file.
%% 
%% Copyright 2003--2007 by Sebastian Lohmeier
%% 
%% This file may be distributed and/or modified under the
%% conditions of the LaTeX Project Public License, either version 1.2
%% of this license or (at your option) any later version.
%% The latest version of this license is in:
%% 
%%    http://www.latex-project.org/lppl.txt
%% 
%% and version 1.2 or later is part of all distributions of LaTeX
%% version 1999/12/01 or later.
%% 
\newcommand\AM@classname{%
  AMpaper%
}
\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{\AM@classname}
  [2007/01/31 v1.0 LaTeX2e
    document class
]
\ClassInfo{\AM@classname}{Dokumentklasse
  des Lehrstuhls fuer Angewandte Mechanik,\MessageBreak%
  Technische Universitaet Muenchen.\MessageBreak%
  Erstellt von Sebastian Lohmeier.}%
\newcommand\baseclass{%
  scrartcl%
}
\PassOptionsToClass{a4paper}{\baseclass}
\PassOptionsToClass{normalheadings}{\baseclass}
\PassOptionsToClass{halfparskip}{\baseclass}
\PassOptionsToClass{fleqn}{\baseclass}
\PassOptionsToPackage{fleqn}{amsmath}
\PassOptionsToPackage{intlimits,sumlimits}{amsmath}
\PassOptionsToClass{pointlessnumbers}{\baseclass}
\PassOptionsToClass{cleardoubleempty}{\baseclass}
\PassOptionsToClass{tablecaptionabove}{\baseclass}
\PassOptionsToClass{abstracton}{\baseclass}
\PassOptionsToPackage{pagesize,headinclude}{typearea}
\newif\ifonline\onlinefalse
\DeclareOption{online}{\onlinetrue}
\newif\ifdraftversion\draftversionfalse
\newif\ifdraftclsmode\draftclsmodefalse
\DeclareOption{draftcls}{\draftclsmodetrue}
\DeclareOption{draft}{\draftversiontrue\draftclsmodetrue}
\newif\ifreview\reviewfalse
\DeclareOption{review}{\reviewtrue}
\DeclareOption{oldstyle}{\PassOptionsToPackage{oldstyle}{AMtools}}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{\baseclass}}
\ProcessOptions*\relax
\LoadClass{\baseclass}
\let\scr@fnt@wrn@pagehead=\@undefined
\let\scr@fnt@wrn@pagefoot=\@undefined
\newcommand\@ForbiddenPackage[1]{%
  \BeforePackage{#1}{%
    \ClassError{\AM@classname}{Verbotenes Paket '#1' geladen!}{%
      Es ist verboten, das Paket '#1' zu benutzen,\MessageBreak
      denn es beeinflusst das Layout oder ist inkompatibel
      mit '\AM@classname'.\MessageBreak
      Diese Fehlermeldung erscheint eine Zeile unter dem Paket.}}}
\@ForbiddenPackage{a4wide}
\@ForbiddenPackage{a4}
\@ForbiddenPackage{a5comb}
\@ForbiddenPackage{a5}
\@ForbiddenPackage{aeguill}
\@ForbiddenPackage{ae}
\@ForbiddenPackage{appendix}
\@ForbiddenPackage{avantgar}
\@ForbiddenPackage{avant}
\@ForbiddenPackage{bm}
\@ForbiddenPackage{bibgerm}
\@ForbiddenPackage{bookman}
\@ForbiddenPackage{caption2}
\@ForbiddenPackage{caption}
\@ForbiddenPackage{chancery}
\@ForbiddenPackage{chappg}
\@ForbiddenPackage{charter}
\@ForbiddenPackage{cmbright}
\@ForbiddenPackage{concmath}
\@ForbiddenPackage{courier}
\@ForbiddenPackage{doublespace}
\@ForbiddenPackage{dropcaps}
\@ForbiddenPackage{eco}
\@ForbiddenPackage{endfloat}
\@ForbiddenPackage{epsfig}
\@ForbiddenPackage{eulervm}
\@ForbiddenPackage{extsizes}
\@ForbiddenPackage{fandyhdr}
\@ForbiddenPackage{figcaps}
\@ForbiddenPackage{floatfig}
\@ForbiddenPackage{floatflt}
\@ForbiddenPackage{fltpage}
\@ForbiddenPackage{french}
\@ForbiddenPackage{geometry}
\@ForbiddenPackage{geomsty}
\@ForbiddenPackage{german}
\@ForbiddenPackage{hangcaption}
\@ForbiddenPackage{hangftn}
\@ForbiddenPackage{hanging}
\@ForbiddenPackage{helvet}
\@ForbiddenPackage{helvetic}
\@ForbiddenPackage{here}
\@ForbiddenPackage{hvmath}
\@ForbiddenPackage{indentfirst}
\@ForbiddenPackage{initials}
\@ForbiddenPackage{lucidabr}
\@ForbiddenPackage{lucidaso}
\@ForbiddenPackage{mathpazo}
\@ForbiddenPackage{mathptm}
\@ForbiddenPackage{mathptmx}
\@ForbiddenPackage{mathsans}
\@ForbiddenPackage{mathtime}
\@ForbiddenPackage{ncntrsbk}
\@ForbiddenPackage{newcent}
\@ForbiddenPackage{ngerman}
\@ForbiddenPackage{palatcm}
\@ForbiddenPackage{palatino}
\@ForbiddenPackage{sansmath}
\@ForbiddenPackage{sectsty}
\@ForbiddenPackage{times}
\@ForbiddenPackage{titlesec}
\@ForbiddenPackage{utopia}
\@ForbiddenPackage{zapfchan}
\@ForbiddenPackage{zefonts}
\RequirePackage{fixltx2e}
\RequirePackage[T1]{fontenc}
\RequirePackage{lmodern}
\RequirePackage[rmdefault]{mathcomp}[2001/01/07]
\RequirePackage{ragged2e}
\RequirePackage{graphicx}[1999/02/16]
\RequirePackage{AMtools}
\RequirePackage{babel}[2001/03/01]
\RequirePackage[headsepline]{scrpage2}
\RequirePackage{microtype}
\RequirePackage{setspace}
\RequirePackage{cite}
\RequirePackage{array}
\RequirePackage{dcolumn}
\RequirePackage{icomma}
\areaset[8mm]{150mm}{250mm}
\begingroup
\@tempdima=\textheight
\advance\@tempdima by -0.6\baselineskip
\textheight0mm
\@whiledim\textheight<\@tempdima
  \do{\global\advance\textheight by \baselineskip}
\endgroup
\ClassInfo{\AM@classname}{%
    These are the values describing the layout:\MessageBreak
    BCOR\space=\space\the\ta@bcor\MessageBreak
    \string\paperwidth\space\space\space\space\space\space=%
                         \space\the\paperwidth\MessageBreak
    \space\string\textwidth\space\space\space\space\space\space=%
                         \space\the\textwidth\MessageBreak
    \space\string\evensidemargin\space=%
                         \space\the\evensidemargin\MessageBreak
    \space\string\oddsidemargin\space\space=%
                         \space\the\oddsidemargin\MessageBreak
    \string\paperheight\space\space\space\space\space=%
                         \space\the\paperheight\MessageBreak
    \space\string\textheight\space\space\space\space\space=%
                         \space\the\textheight\MessageBreak
    \space\string\topmargin\space\space\space\space\space\space=%
                         \space\the\topmargin\MessageBreak
    \space\string\headheight\space\space\space\space\space=%
                         \space\the\headheight\MessageBreak
    \space\string\headsep\space\space\space\space\space\space\space\space=%
                         \space\the\headsep\MessageBreak
    \space\string\topskip\space\space\space\space\space\space\space\space=%
                         \space\the\topskip\MessageBreak
    \space\string\footskip\space\space\space\space\space\space\space=%
                         \space\the\footskip\MessageBreak
    \space\string\baselineskip\space\space\space=%
                         \space\the\baselineskip\MessageBreak}
\clubpenalty=100 % (AMclasses bisher: 5000)
\widowpenalty=400 % (AMclasses bisher: 5000)
\displaywidowpenalty=50 % (AMclasses bisher: 5000)
\brokenpenalty=400
\relpenalty=700
\binoppenalty=2000
\setkomafont{descriptionlabel}{%
  \normalcolor\normalfont\rmfamily\bfseries}
\setkomafont{pagehead}{\normalcolor\normalfont\small\sffamily}
\setkomafont{pagenumber}{\usekomafont{pagehead}}
\pagestyle{scrheadings}
\clearscrheadfoot
\ihead[]{\headmark}
\ohead[]{\pagemark}
\ifdraftclsmode\else\cfoot[\pagemark]{}\fi
\if@twoside
  \automark[subsection]{section}
\else
  \automark{section}
\fi
\newcommand\sectionname{}
\addto\extrasngerman{%
  \renewcommand{\bibname}{Literatur}
  \renewcommand{\figurename}{Bild}
  \renewcommand\sectionname{Abschnitt}}
\addto\extrasenglish{%
  \renewcommand\sectionname{section}}
\iffalse
\mathchardef\mathcomma\mathcode`\,
\mathcode`\,=32768 % "8000

\begingroup
\catcode`\_=12
\catcode`\,=\active
\@firstofone{%
  \endgroup
  %
  \edef\DC@char{\meaning_}
  \def\DC@parse#1 _{\def\DC@parse##1#1 ##2##3\delimiter{##2}}
  \expandafter\DC@parse\DC@char
  \toks@={\expandafter\DC@parse\meaning\DC@char}
  \edef\DC@test{9<2\the\toks@ \DC@char.\delimiter}
  %
  \DeclareRobustCommand\ActiveComma{\futurelet\DC@char\DC@docomma}
  \def\DC@docomma{%
    \ifnum\DC@test \mathord \fi
    \mathcomma
  }
  \let,\ActiveComma
}
\fi
\renewenvironment{thebibliography}[1]{%
  \ifx\bibpreamble\relax\else\ifx\bibpreamble\@empty\else
    \setchapterpreamble{\bibpreamble}%
  \fi\fi
  \bib@heading%
  \list{\@biblabel{\@arabic\c@enumiv}}{%
    \settowidth\labelwidth{\@biblabel{#1}}%
    \leftmargin\labelwidth
    \advance\leftmargin\labelsep
    \@openbib@code
    \usecounter{enumiv}%
    \let\p@enumiv\@empty
    \renewcommand*\theenumiv{\@arabic\c@enumiv}%
  }%
  \sloppy\clubpenalty4000\widowpenalty4000%
  \setlength\RaggedRightRightskip{\z@\@plus 6em}%
  \small\RaggedRight
  \sfcode`\.=\@m
}{
  \def\@noitemerr
  {\@latex@warning{Empty `thebibliography' environment}}%
  \endlist
}
\renewcommand*\newblock{\ }
\def\@citex[#1]#2{%
  \let\@citea\@empty
  \@cite{\@for\@citeb:=#2\do
    {\@citea\def\@citea{,\,}%
     \edef\@citeb{\expandafter\@firstofone\@citeb\@empty}%
     \if@filesw\immediate\write\@auxout{\string\citation{\@citeb}}\fi
     \@ifundefined{b@\@citeb}{\mbox{\reset@font\bfseries ?}%
       \G@refundefinedtrue
       \@latex@warning
         {Citation `\@citeb' on page \thepage \space undefined}}%
       {\hbox{\csname b@\@citeb\endcsname}}}}{#1}}
\newcolumntype{.}[1]{D{.}{.}{#1}}
\newcolumntype{,}[1]{D{,}{\mathord\mathcomma}{#1}}
\newcommand{\PreserveBackslash}[1]{\let\@tempa=\\#1\let\\=\@tempa}
\newcolumntype{L}[1]{>{\PreserveBackslash\RaggedRight\hspace{\z@}}p{#1}}
\newcolumntype{C}[1]{>{\PreserveBackslash\Centering\hspace{\z@}}p{#1}}
\newcolumntype{R}[1]{>{\PreserveBackslash\RaggedLeft\hspace{\z@}}p{#1}}
\renewcommand\fps@figure{tp}
\renewcommand\fps@table{tp}
\renewcommand{\floatpagefraction}{0.75}  % Default: 0.5
\renewcommand{\topfraction}{0.75}        % Default: 0.7
\renewcommand{\bottomfraction}{0.35}     % Default: 0.3
\renewcommand{\textfraction}{0.2}       % Default: 0.2
\setcounter{topnumber}{2}               % Default: 2
\setcounter{bottomnumber}{1}            % Default: 1
\setcounter{totalnumber}{3}             % Default: 3
\g@addto@macro\end@float{%
  \ifnum\@floatpenalty<\z@
    \ifnum\@floatpenalty<-\@Mii
    \else
      \ifhmode
        \if@ignore
          \penalty\@M
          \hskip\z@skip
        \fi
      \fi
    \fi
  \fi}
\setlength\floatsep{18\p@ \@plus 10\p@ \@minus 2\p@}
\setlength\textfloatsep{14\p@ \@plus 20\p@ \@minus 4\p@}
\setlength\@fptop{0\p@ \@plus 1fil}
\setlength\@fpsep{18\p@ \@plus 1fil}
\setlength\@fpbot{0\p@ \@plus 2fil}
\BeforePackage{float}{%
  \newcommand*\AM@float@addtolists{}%
  \let\AM@float@addtolists=\float@addtolists
  \let\float@addtolists=\undefined}
\AfterPackage{float}{%
  \let\AM@float@addtolists=\float@addtolists
  \renewcommand\float@addtolists[1]{%
    \AM@float@addtolists{#1}%
    \AM@float@addtolists{#1}}%
  \renewcommand*\listof[2]{%
    \begingroup%
      \@ifundefined{ext@#1}{%
        \float@error{#1}}{%
        \@namedef{l@#1}{\l@figure}%
        \if@twocolumn
          \@restonecoltrue\onecolumn
        \else
          \@restonecolfalse
        \fi
        \float@listhead{#2}%
        \small
        \setlength{\parskip}{\z@}%
        \@starttoc{\@nameuse{ext@#1}}%
        \if@restonecol\twocolumn\fi
      }%
    \endgroup
  }%
  \floatstyle{komabelow}}
\ifpdfoutput{%
  \DeclareGraphicsExtensions{.pdf,.png,.jpg}}{%
  \DeclareGraphicsExtensions{.eps}}
\setcaphanging
\setkomafont{caption}{\normalfont\normalcolor\small\RaggedRight}
\setkomafont{captionlabel}{\usekomafont{sectioning}}
\newcommand*\@floatfont{\normalfont\normalcolor\normalsize}
\newcommand*\scr@fnt@float{\@floatfont}
\@ifpackageloaded{setspace}{%
  \def\@xfloat #1[#2]{%
    \latex@xfloat #1[#2]%
    \def\baselinestretch{\setspace@singlespace}%
    \scr@fnt@float}}{%
  \def\@floatboxreset{%
    \reset@font
    \scr@fnt@float
    \@setminipage}}
\setkomafont{float}{\normalfont\normalcolor\small}
\AfterPackage{longtable}{%
  \newcommand\AM@longtable{}
  \let\AM@longtable\longtable
  \renewcommand\longtable{\small\AM@longtable}}
\renewcommand*\listoffigures{%
  \begingroup%
    \if@twocolumn
      \@restonecoltrue\onecolumn
    \else
      \@restonecolfalse
    \fi
    \lof@heading%
    \small
    \setlength{\parskip}{\z@}%
    \setlength{\parindent}{\z@}%
    \setlength{\parfillskip}{\z@ \@plus 1fil}%
    \@starttoc{lof}%
    \if@restonecol\twocolumn\fi
  \endgroup}
\renewcommand*\listoftables{%
  \begingroup
    \if@twocolumn
      \@restonecoltrue\onecolumn
    \else
      \@restonecolfalse
    \fi
    \lot@heading%
    \small
    \setlength{\parskip}{\z@}%
    \setlength{\parindent}{\z@}%
    \setlength{\parfillskip}{\z@ \@plus 1fil}%
    \@starttoc{lot}%
    \if@restonecol\twocolumn\fi
  \endgroup}
\deffootnote{1.5em}{1.5em}{\thefootnotemark\ }
\addtokomafont{footnote}{\normalfont\RaggedRight}
\setlength{\skip\footins}{10\p@ \@plus 4\p@ \@minus 2\p@}
\renewcommand{\author}[1]{\gdef\@author{\def\and{,\ }#1}}
\newcommand*{\@amsubtitle}{}
\newcommand{\amsubtitle}[1]{\gdef\@amsubtitle{#1}}
\renewcommand*{\@maketitle}{%
    \clearpage
    \ifreview\singlespacing\fi
    \let\footnote\thanks
    % Hintergrund
    \null\vspace*{-10mm}\hspace*{-35mm}
    \begin{picture}(0,0)
      \put(0,0){\parbox[t][\textheight][c]{\textwidth}{%
          \includegraphics{AMpaperTitlepage}}}
    \end{picture}\par
    \null\vskip 8ex
    \begin{center}\sffamily%
      \ifx\@subject\@empty \else
        {\large\@subject\par}
        \bigskip
      \fi
      \textcolor{AMBlue}{\bfseries\LARGE\@title}\par
      \ifx\@amsubtitle\@empty \else
        \textcolor{AMBlue}{\large\@amsubtitle}\par
      \fi
      \bigskip
      {\Large\@author}\par
      \smallskip
      {\Large%
        \ifdraftclsmode
          {\AM@PrelimStyle \"Ubersetzt: \today/\now}
        \else
          \@date
        \fi}%
    \end{center}%
  \par
  \vskip 2em}
\ifdraftclsmode
  \AtBeginDocument{\setlength\overfullrule{5pt}}
  \RequirePackage[color]{showkeys}
  \definecolor{refkey}{rgb}{0.05,0.54,0}
  \definecolor{labelkey}{rgb}{0.05,0.54,0}
  \newcommand*\now{{%
      \def\Time{3}%
      \def\Hour{4}%
      \def\Minute{5}%
      \count\Time=\time\relax
      \count\Hour=\count\Time\relax
      \divide\count\Hour by 60\relax%%hours past midnight
      \count\Minute=\count\Hour\relax
      \multiply\count\Minute by -60\relax
      \advance\count\Minute by \count\Time\relax %%minutes past hour
      \the\count\Hour\relax:%
      \ifnum\count\Minute<10
        0%
      \fi
      \the\count\Minute\relax}}
  \newcommand{\AM@PrelimStyle}{\color{red}\ttfamily}
  \newcommand{\AM@PrelimWords}{%
    \jobname.tex\space(\AM@classname.cls);\ Seite\,\thepage;\
    \"Ubersetzt:\space\today/\now}
  \ofoot[{\AM@PrelimStyle\footnotesize\AM@PrelimWords}]{%
    {\AM@PrelimStyle\footnotesize\AM@PrelimWords}}
  \AtEndDocument{\ClassWarningNoLine{\AM@classname}{%
      Bitte nicht vergessen,\MessageBreak
      in der endgueltigen Version des Dokuments\MessageBreak
      die Option <draftcls> zu entfernen!}}
\fi
\ifreview
  \AtBeginDocument{\doublespacing}
  \AtEndDocument{\ClassWarningNoLine{\AM@classname}{%
      Bitte nicht vergessen,\MessageBreak
      in der endgueltigen Version des Dokuments\MessageBreak
      die Option <review> zu entfernen!}}
\fi
\allowdisplaybreaks
\def\maketag@@@#1{\hbox{\m@th#1}}
\ifpdfoutput{%
  \ifonline
    \RequirePackage[bookmarks=true,breaklinks=true,colorlinks=true,%
      citecolor=AMBlue,filecolor=AMBlue,linkcolor=AMBlue,%
      pagecolor=AMBlue,urlcolor=AMBlue]{hyperref}
  \else
    \RequirePackage[bookmarks=true,colorlinks=true,%
      citecolor=black,filecolor=black,linkcolor=black,%
      pagecolor=black,urlcolor=black]{hyperref}
  \fi
  }{}
\newenvironment{enumcont}{%
  \renewcommand{\usecounter}[1]{%
    \@nmbrlisttrue\def\@listctr{##1}}
  \enumerate}{\endlist}
\newlength{\Descriptionlabelwidth}
\setlength{\Descriptionlabelwidth}{6em}
\newcommand{\Descriptionlabel}[1]{%
  \settowidth{\@tempdima}{\descfont #1}%
  \ifdim\@tempdima>\Descriptionlabelwidth%
    \parbox[b]{\Descriptionlabelwidth}{\makebox[0pt][l]{\descfont #1}\\}%
  \else%
    \makebox[0pt][l]{\descfont #1}
  \fi}
\newenvironment{Description}[1][\Descriptionlabelwidth]
{\begin{list}{}{%
    \setlength{\labelwidth}{#1}%
    \setlength{\labelsep}{\z@}%
    \setlength{\itemindent}{\z@}%
    \setlength{\leftmargin}{\labelwidth}
    \addtolength{\leftmargin}{\labelsep}%
    \let\makelabel\Descriptionlabel}}{\end{list}}
\newcommand*\@advancedfont{\footnotesize}
\newcommand*\scr@fnt@advancedtext{\@advancedfont}
\newcommand*\@advancednamefont{\bfseries}
\newcommand*\scr@fnt@advancedname{\@advancednamefont}
\newcommand\advancedspacing{0.8}
\newcommand*\noadvancedname{\gdef\advancedname{}}
\noadvancedname
\newenvironment{advanced}{%
  \let\size@subsection\@advancedfont
  \let\size@subsubsection\@advancedfont
  \let\size@paragraph\@advancedfont
  \let\size@subparagraph\@advancedfont
  \begin{addmargin}{\leftmargini}
    \begin{spacing}{\advancedspacing}
    \@advancedfont
     \ifx\advancedname\@empty\else
       {\@advancednamefont\advancedname}\quad%
     \fi
    \ignorespaces}{%
  \end{spacing}\end{addmargin}}
  \newcommand{\chref}[1]{\chaptername~\ref{#1}}
  \newcommand{\secref}[1]{\sectionname~\ref{#1}}
  \newcommand{\appref}[1]{\appendixname~\ref{#1}}
  \newcommand{\figref}[1]{\figurename~\ref{#1}}
  \newcommand{\tabref}[1]{\tablename~\ref{#1}}
\newcommand*{\ProfUlbrich}{%
  Prof. \mbox{Dr.-Ing.} \mbox{Dr.-Ing.\,habil.} Heinz Ulbrich\xspace}
\newcommand*{\ProfPfeiffer}{%
  Prof. \mbox{Dr.-Ing.} \mbox{Dr.-Ing.\,E.h.} \mbox{Dr.-Ing.\,h.c.\,(RAS)}
  Friedrich Pfeiffer i.\,R.\xspace}
\providecommand*\GetFileInfo[1]{%
  \def\filename{#1}%
  \def\@tempb##1 ##2 ##3\relax##4\relax{%
    \def\filedate{##1}%
    \def\fileversion{##2}%
    \def\fileinfo{##3}}%
  \edef\@tempa{\csname ver@#1\endcsname}%
  \expandafter\@tempb\@tempa\relax? ? \relax\relax}
\endinput
%%
%% End of file `AMpaper.cls'.
