function varargout = GUI(varargin)
% GUI MATLAB code for GUI.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI

% Last Modified by GUIDE v2.5 12-Sep-2014 10:44:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before GUI is made visible.
function GUI_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI (see VARARGIN)

% Choose default command line output for GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


%% --- Executes on button press in pushbutton1.
%      Data Read&Display According File Type
function pushbutton1_Callback(hObject, ~, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%open SetPathLocal.m;
diary;
run SetPathLocal;
cla reset;
%cla(axes2);
%cla(axes3);
%cla(axes4);
%cla(axes5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   UI select file and import into database
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[filename, pathname,filterindex]=uigetfile(...
    {'*.stl','Model file(*.stl)';...
    '*.xyz','MeshLab file(*.xyz)';...
    '*.txt','DIY file(*.txt)';
    '*.*','All file'},...
    'File Selector');

%check if file selection cancelled or not
if isequal(filename,0)
    disp('selected cancelled');
    diary off;
    fid=fopen('diary','r');
    while feof(fid)==0
        str=fgetl(fid);
        pause(0.2);
        set(handles.edit1,'string',str);
    end
    fclose(fid);
    delete('diary');

else
    disp(['selected: ',fullfile(pathname,filename)]);
    % check the file suffix to judege the file read method
    [~,~,suffix] = fileparts(filename);
    TF1=strcmp('.txt',suffix);
    TF2=strcmp('.stl',suffix);
    TF3=strcmp('.xyz',suffix);

    %choose the .txt file
    if(TF1==1)
        matr_of_stl=importdata(filename);
        [B,~,~]=unique(matr_of_stl,'rows');
        [m,n]=size(B);
        x=zeros(m,1);
        y=zeros(m,1);
        z=zeros(m,1);
        for i=1:m;
            for j=1:n;
                x(i)=B(i,1);
                y(i)=B(i,2);
                z(i)=B(i,3);
            end
        end
    end

    %choose the .stl file
    if(TF2==1)
        fid = fopen(filename,'r');    
        if ~isempty(ferror(fid))
            error(lasterror); %#ok
        end
    
        M = fread(fid,inf,'uint8=>uint8');
        fclose(fid);
    
        %[f,v,n] = stlbinary(M);
        %F = [];
        %V = [];
        %N = [];
    
        if length(M) < 84
            error('MATLAB:stlread:incorrectFormat', ...
                'Incomplete header information in binary STL file.');
        end
    
        % Bytes 81-84 are an unsigned 32-bit integer specifying the number of faces
        % that follow.
        numFaces = typecast(M(81:84),'uint32');
        %numFaces = double(numFaces);
        if numFaces == 0
            warning('MATLAB:stlread:nodata','No data in STL file.');
            return
        end
    
        T = M(85:end);
        %F = NaN(numFaces,3);
        V = NaN(3*numFaces,3);
        %N = NaN(numFaces,3);
    
        numRead = 0;
        while numRead < numFaces
            % Each facet is 50 bytes
            %  - Three single precision values specifying the face normal vector
            %  - Three single precision values specifying the first vertex (XYZ)
            %  - Three single precision values specifying the second vertex (XYZ)
            %  - Three single precision values specifying the third vertex (XYZ)
            %  - Two unused bytes
            i1    = 50 * numRead + 1;
            i2    = i1 + 50 - 1;
            facet = T(i1:i2)';
        
            %n  = typecast(facet(1:12),'single');
            v1 = typecast(facet(13:24),'single');
            v2 = typecast(facet(25:36),'single');
            v3 = typecast(facet(37:48),'single');
        
            %n = double(n);
            v = double([v1; v2; v3]);
        
            % Figure out where to fit these new vertices, and the face, in the
            % larger F and V collections.        
            fInd  = numRead + 1;        
            vInd1 = 3 * (fInd - 1) + 1;
            vInd2 = vInd1 + 3 - 1;
        
            V(vInd1:vInd2,:) = v;
            %F(fInd,:)        = vInd1:vInd2;
            %N(fInd,:)        = n;  
            numRead = numRead + 1;
        end
        VF=unique(V,'rows');
        x=VF(:,1);
        y=VF(:,2);
        z=VF(:,3);
    end

    %choose the .xyz file
    if(TF3==1)
        fid = fopen(filename,'r+');
        disp(fid);
        if ~isempty(ferror(fid))
            error(lasterror); %#ok
        end
    
        i=0;
        j=1;
        tic;
        while ~feof(fid)
            i=i+1;
            fgetl(fid);
        end
        frewind(fid);
        V=zeros(i,3);
        while ~feof(fid)
            tline = fgetl(fid);
            V(j,:) = str2num(tline);
            j=j+1;
        end
        fclose(fid);
        t=toc;
        disp(t);
        x=V(:,1);
        y=V(:,2);
        z=V(:,3);
    
    end
    handles.x=x;
    handles.m=size(x);
    disp(size(x));
    handles.y=y;
    handles.z=z;
    
    mfig=plot3(handles.axes1,handles.x,handles.y,handles.z,'.r');
    h=mfig;
    set(h,'ButtonDownFcn',{@myCallback,handles});
    %handles.h=h;

    mfig1=plot(handles.axes2,handles.x,handles.y,'.g');
    h1=mfig1;
    set(h1,'ButtonDownFcn',{@myCallback1,handles});
    %handles.h1=h1;
    %guidata(hObject,handles);

    mfig2=plot(handles.axes3,handles.y,handles.z,'.g');
    h2=mfig2;
    set(h2,'ButtonDownFcn',{@myCallback2,handles});
    handles.h2=h2;
    %guidata(hObject,handles);

    mfig3=plot(handles.axes4,handles.x,handles.z,'.g');
    h3=mfig3;
    set(h3,'ButtonDownFcn',{@myCallback3,handles});
    %handles.h3=h3;
    guidata(hObject,handles);
    diary off;
    fid=fopen('diary','r');
    while feof(fid)==0
        str=fgetl(fid);
        pause(0.2);
        set(handles.edit1,'string',str);
    end
    fclose(fid);
    delete('diary');
end
return;

%% --- Executes on button press in pushbutton2.
%      Plane Feature Test
function pushbutton2_Callback(~, ~, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%run PlaneFeature;
%cd ..;
global px;
global py;
global pz;
global couter;
global PreX;
global PreY;
global PreZ;
global X;
global Y;
global Z;
%global planepara;

global planepara;
[X,Y,Z,planepara]=PlaneFeature(px,py,pz,couter);
PreX=X;PreY=Y;PreZ=Z;
mfig=plot3(handles.axes5,PreX,PreY,PreZ,'+m');
h5=mfig;
set(h5,'ButtonDownFcn',{@myCallback4,handles});
text(X(10)-1000,Y(10)-1000,Z(10)-1000,[num2str(planepara(1)),'*x+',num2str(planepara(2)),'*y+',num2str(planepara(3)),'*z+',num2str(planepara(4)),'=0']);
set(handles.edit2,'string',['PlaneFlatness = ',num2str(planepara(6))]);
return

%% --- Executes on button press in pushbutton3.
%      Parallelism Between Two Recognized Plane
function pushbutton3_Callback(~, ~, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%%%%%%%%%%the base plane
%set(handles.edit2,'string','Confirm The Base Plane');
global planepara;
%disp(planepara)
global px
global py
global pz
global couter
global PreX;
global PreY;
global PreZ;
global X;
global Y;
global Z;
global res;
%PreX=px;PreY=py;PreZ=pz;
mfig=plot3(handles.axes5,X,Y,Z,'+r');
hold(handles.axes5);
[parallelvalue,res]=ParallismTest(planepara,px,py,pz,couter);
%disp(parallelvalue)
num=sum(res.CS(:));
SX=zeros(1,num);
SY=zeros(1,num);
SZ=zeros(1,num);
j=1;
for i=1:couter
    if(res.CS(i)==1)
        SX(j)=px(i);
        SY(j)=py(i);
        SZ(j)=pz(i);
        j=j+1;
    end
end
plot3(handles.axes5,SX,SY,SZ,'.y')
set(handles.edit2,'string',['Parallel = ',num2str(parallelvalue)]);
PreX=[X,SX];
PreY=[Y,SY];
PreZ=[Z,SZ];
h5=mfig;
set(h5,'ButtonDownFcn',{@myCallback4,handles});
return



% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%% --- Executes on button press in pushbutton6.
%      Circle feature test
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global px
global py
global pz
global couter
r=5;
[CS,Para,R,Center]=CircleTest(px,py,pz,couter,r);
%disp(Para);
num=sum(CS(:));
SX=zeros(1,num);
SY=zeros(1,num);
SZ=zeros(1,num);
global PreX;
global PreY;
global PreZ;
j=1;
for i=1:couter
    if(CS(i)==1)
        SX(j)=px(i);
        SY(j)=py(i);
        SZ(j)=pz(i);
        j=j+1;
    end
end
mfig=plot3(handles.axes5,SX,SY,SZ,'.y');
set(handles.edit2,'string',['Circle Radius = ',num2str(R),'CircleParameter = ',num2str(Center)]);
PreX=SX;
PreY=SY;
PreZ=SZ;
h5=mfig;
set(h5,'ButtonDownFcn',{@myCallback4,handles});
return

function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


%% --- Executes on button press in pushbutton8.
% Out the file
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
resultfile=fopen('.\Data\resultfile.txt','wt');
global couter;
global px;
global py;
global pz;
for i=1:couter
    fprintf(resultfile,'%.6f %.6f %.6f\n',[px(i),py(i),pz(i)]);
end
fclose(resultfile);

global X;
global Y;
global Z;
[~,m]=size(X);
if(isempty(X))
    return
else
    resultfile=fopen('.\Data\result_plane_file.txt','wt');
    for i=1:m
        fprintf(resultfile,'%.6f %.6f %.6f\n',[X(i),Y(i),Z(i)]);
    end
    fclose(resultfile);
end

if(isempty(X))
    return
else
    resultfile=fopen('.\Data\Parallel_result_file.txt','wt');
    for i=1:couter
        fprintf(resultfile,'%.6f %.6f %.6f\n',[X(i),Y(i),Z(i)]);
    end
    fclose(resultfile);
end
return


% --- Executes on mouse press over axes background.
function axes2_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on mouse press over axes background.
function axes3_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on mouse press over axes background.
function axes4_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Self-definition callback function for point selection XYZ
function myCallback(hObject, eventdata, handles)
% hObject    handle to axesCleaning1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%x-y perspective
figure('Name','Preview','NumberTitle','off');
plot3(handles.x,handles.y,handles.z,'.m')
xlabel('X');
ylabel('Y');
zlabel('Z');
return
%% Self-definition callback function for point selection XOY
function myCallback1(hObject, eventdata, handles)
% hObject    handle to axesCleaning1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%x-y perspective
figure(1)
plot(handles.x,handles.y,'.m')
xlabel('X');
ylabel('Y');
hh1=imrect;
if ~isempty(hh1)
    global pos1;
    pos1=getPosition(hh1);
    %handles.pos1=pos1;
    %guidata(hObject,handles);
else
    set(handles.edit2,'string','Retry the XOY selection');
end
pause(0.5)
close
return
%% Self-definition callback function for point selection YOZ
function myCallback2(hObject, eventdata, handles)
% hObject    handle to axesCleaning1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%y-z perspective
figure(2);
plot(handles.y,handles.z,'.m')
xlabel('Y');
ylabel('Z');
hh2=imrect;
if ~isempty(hh2)
    global pos2;
    pos2=getPosition(hh2);
    %handles.pos2=pos2;
    %guidata(hObject,handles);
else
    set(handles.edit2,'string','Retry the YOZ selection');
end
pause(0.5)
close
return
%% Self-definition callback function for point selection XOZ
function myCallback3(hObject, eventdata, handles)
% hObject    handle to axesCleaning1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%x-z perspective
figure(3);
plot(handles.x,handles.z,'.m')
xlabel('X');
ylabel('Z');
hh3=imrect;
if ~isempty(hh3)
    global pos3;
    pos3=getPosition(hh3);
    %handles.pos3=pos3;
    %guidata(hObject,handles);
else
    set(handles.edit2,'string','Retry the XOZ selection');
end
pause(0.5)
close
return
%% Self-definition callback function for point selection XYZ
function myCallback4(hObject, eventdata, handles)
% hObject    handle to axesCleaning1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%
figure('Name','Preview','NumberTitle','off');
global PreX;
global PreY;
global PreZ;
plot3(PreX,PreY,PreZ,'.m')
xlabel('X');
ylabel('Y');
zlabel('Z');
return


%% --- Executes on button press in pushbutton9.
% Get the Compatible Point
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num_of_selected_point=1;
picx=zeros(1,10);
picy=zeros(1,10);
picz=zeros(1,10);
global pos1;
global pos2;
global pos3;
%pos1=handles.pos1;
%pos2=handles.pos2;
xmin=min([pos1(1),pos3(1)]);
xmax=max([pos1(1)+pos1(3),pos3(1)+pos1(3)]);
ymin=min([pos1(2),pos2(1)]);
ymax=max([pos1(2)+pos1(4),pos2(1)+pos2(3)]);
zmin=min([pos2(2),pos3(2)]);
zmax=max([pos2(2)+pos2(4),pos3(2)+pos3(4)]);
%disp(handles.x);
for i=1:handles.m
    if ((handles.x(i)>xmin) && (handles.x(i)<xmax))
        if ((handles.y(i)>ymin) && (handles.y(i)<ymax))
            if ((handles.z(i)>zmin) && (handles.z(i)<zmax))
                picx(num_of_selected_point)=handles.x(i);
                picy(num_of_selected_point)=handles.y(i);
                picz(num_of_selected_point)=handles.z(i);
                num_of_selected_point=num_of_selected_point+1;
            end
        end
    end
end
global couter;
couter=num_of_selected_point-1;
global px;
global py;
global pz;
px=picx(:);
py=picy(:);
pz=picz(:);
global PreX;
global PreY;
global PreZ;
PreX=px;PreY=py;PreZ=pz;
mfig=plot3(handles.axes5,PreX,PreY,PreZ,'.k');
h5=mfig;
set(h5,'ButtonDownFcn',{@myCallback4,handles});
return;

%% --- Executes on button press in pushbutton10.
% Clear All of The Figure
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%cla;
%reset(gcf);
cla(handles.axes1);
cla(handles.axes2);
cla(handles.axes3);
cla(handles.axes4);
cla(handles.axes5);
return

% --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on mouse press over axes background.
function axes5_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
