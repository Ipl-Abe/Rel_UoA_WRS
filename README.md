# Rel_UoA_WRS

## インストール  
### 前提条件  
1. OpenRTMがインストールされていること  
2. AGXがインストールされているいること(AISTSimulator用のプロジェクトも作成済み/2019/09/29)  
3. 以下のoptionが付加されていること  
* BUILD_AGX_BODYEXTENSION_PLUGIN  
* BUILD_AGX_DYNAMICS_PLUGIN  
* BUILD_COMPETITION_PLUGIN  
* BUILD_CORBA_PLUGIN  
* BUILD_OPENRTM_PLUGIN  
* BUILD_OPENRTM_SAMPLES  
* BUILD_WRS2018  
* ENABLE_CORBA  






### コマンド
    cd ~/chorenoid/ext   
    git clone https://github.com/Ipl-Abe/Rel_UoA_WRS.git  
    cd ~/choreonoid/build   
    ccmake ..   
    BUILD_REL_UOA_WRS_OPENRTMをONにする  
    make   
    
## プロジェクトの起動(for AIST)
    cd ~/choreonoid/build   
    bin/choreonoid ../sample/WRS/script/T2-AizuSpiderDS-RTM.py 
    または
    CNOID_USE_GLSL=0 bin/choreonoid ../sample/WRS/script/T2-AizuSpiderDS-RTM.py
## プロジェクトの起動(for AGX)
    cd ~/choreonoid/build   
    bin/choreonoid ../sample/WRS/script/T2-AizuSpiderDA-RTM.py 
    または
    CNOID_USE_GLSL=0 bin/choreonoid ../sample/WRS/script/T2-AizuSpiderDA-RTM.py
##(逐次更新予定)
