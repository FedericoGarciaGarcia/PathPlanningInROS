/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include "../include/my_astar_planner_2/myAstarPlanner.h"
#include <pluginlib/class_list_macros.h>

//para pintar puntos
#include <visualization_msgs/Marker.h>

// para debugging
#include <sstream>
#include <string>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(myastar_planner::MyastarPlanner, nav_core::BaseGlobalPlanner)

namespace myastar_planner {

list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID);
bool isContains(list<coupleOfCells> & list1, int cellID);

MyastarPlanner::MyastarPlanner()
    : costmap_ros_(NULL), initialized_(false) {
}

MyastarPlanner::MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false) {
    initialize(name, costmap_ros);
}

void MyastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if(!initialized_) {
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        ros::NodeHandle private_nh("~/" + name);

        //vamos a asumir estos parámetros, que no es necesario enviar desde el launch.
        private_nh.param("step_size", step_size_, costmap_->getResolution());
        private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);

        //el plan se va a publicar en el topic "planTotal"
        plan_pub_ = private_nh.advertise<nav_msgs::Path>("planTotal",1);
        //los puntos del espacio de búsqueda se visualizan en "visualization_marker"
        marker_Open_publisher = private_nh.advertise<visualization_msgs::Marker>("open_list", 1000);
        marker_Closed_publisher = private_nh.advertise<visualization_msgs::Marker>("closed_list", 1000);
        marker_Goals_publisher = private_nh.advertise<visualization_msgs::Marker>("goals_markers", 1000);

        initialized_ = true;

        world_model_ = 0;

        inicializar();
    }
    else
        ROS_WARN("This planner has already been initialized... doing nothing");
}


// Devuelve true si hay obstaculo; false en caso contrario
bool MyastarPlanner::esObstaculo(unsigned int indice) {
    if(!initialized_) {
        ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
        return false;
    }

    // Solo inicializar la primera vez
    if(footprint.size() == 0)
        footprint = costmap_ros_->getRobotFootprint();

    //if we have no footprint... do nothing
    if(footprint.size() < 3)
        return false;

    //check if the footprint is legal

    // Inicializar si es 0
    if(world_model_ == 0)
    world_model_ = new base_local_planner::CostmapModel(*costmap_);

    // Obtener posicion
    unsigned int x_i, y_i;

    // Obtener las coordenadas de mapa mx,my dado el indice de la celda
    costmap_->indexToCells(indice, x_i, y_i);

    // Comprobar diferentes angulos, si es mayor que OBSTACULO devolver true
    // CONVERTIR A RADIANES
    if(world_model_->footprintCost(x_i, y_i, 0,          footprint) >= OBSTACULO) return true;
    if(world_model_->footprintCost(x_i, y_i, PI/2,       footprint) >= OBSTACULO) return true;
    if(world_model_->footprintCost(x_i, y_i, PI,         footprint) >= OBSTACULO) return true;
    if(world_model_->footprintCost(x_i, y_i, 3.0*PI/2.0, footprint) >= OBSTACULO) return true;

    // No es obstaculo false
    return false;
}

bool MyastarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {

//***********************************************************
// Inicio de gestion de ROS
//***********************************************************
    if(!initialized_) {
        ROS_ERROR("The astar planner has not been initialized, please call initialize() to use the planner");
        return false;
    }

    ROS_DEBUG("MyastarPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    closedList.clear();
    openList.clear();

    // Inicializar matriz
    // Tamano del mapa en celdas
    const int tama_x = costmap_->getSizeInCellsX();
    const int tama_y = costmap_->getSizeInCellsY();

    for(int i=0; i<tama_y; i++)
        for(int j=0; j<tama_x; j++)
            m[i][j] = LIBRE;

    //obtenemos el costmap global  que está publicado por move_base.
    costmap_ = costmap_ros_->getCostmap();

    //Obligamos a que el marco de coordenadas del goal enviado y del costmap sea el mismo.
    //esto es importante para evitar errores de transformaciones de coordenadas.
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
        ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                  costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
        return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    //obtenemos la orientación start y goal en start_yaw y goal_yaw.
    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);


    /**************************************************************************/
    /*************** HASTA AQUÍ GESTIÓN DE ROS *********************************/
    /****************************************************************************/

    //pasamos el goal y start a estructura coupleOfCells
    coupleOfCells cpstart, cpgoal;
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;

    unsigned int mgoal_x, mgoal_y;
    costmap_->worldToMap(goal_x,goal_y,mgoal_x, mgoal_y);

    cpgoal.index = *m_punteros[mgoal_y][mgoal_x].i; //getIndex(mgoal_x, mgoal_y);
    cpgoal.parent=0;
    cpgoal.gCost=0;
    cpgoal.hCost=0;
    cpgoal.fCost=0;

    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    unsigned int mstart_x, mstart_y;
    costmap_->worldToMap(start_x,start_y, mstart_x, mstart_y);
    cpstart.index  = *m_punteros[mstart_y][mstart_x].i;
    cpstart.parent = cpstart.index;
    cpstart.gCost  = 0;
    cpstart.hCost  = MyastarPlanner::calculateHCost(cpstart.index, cpgoal.index);

    ROS_INFO("Inicio: %f, %f", start_x, start_y);
    ROS_INFO("Objetivo: %f, %f", goal_x, goal_y);

    // Comprobar que no es obstaculo
    // Si es obstaculo hacer al nodo final el inicial
    if(costmap_->getCost(mgoal_x, mgoal_y) >= OBSTACULO) {
        ROS_INFO("CAMINO CANCELADO. Objetivo es un obstaculo.");

        goal_x  = start_x;
        goal_y  = start_y;
        mgoal_x = mstart_x;
        mgoal_y = mstart_y;
        cpgoal  = cpstart;
    }

    //ROS_INFO("GOAL: %d, %d", mgoal_x, mgoal_y);

    /* [1] Meter nodo de la posición del robot en ABIERTOS. */
    MyastarPlanner::openList.push(cpstart);

    //ROS_INFO("Inserto en Abiertos: %d", cpstart.index );
    //ROS_INFO("Index del goal: %d", cpgoal.index );

    /**************************************************************************/
    /*************** GESTIÓN VISUALIZACIÓN PUNTOS DE ABIERTOS Y CERRADOS********/
    /****************************************************************************/

    visualization_msgs::Marker points; // definida en la clase como markers_OpenList
    inicializaMarkersPoints(markers_OpenList,"openList", 0,0.0f,1.0f,0.0f);
    inicializaMarkersPoints(markers_ClosedList,"closedList", 1,1.0f,0.0f,0.0f);
    inicializaMarkersLine_List(markers_Goals, "goals", 2, 0.0f, 0.0f,1.0f);

    limpiaMarkers(marker_Open_publisher, markers_ClosedList);
    limpiaMarkers(marker_Closed_publisher, markers_OpenList);

    /**************************************************************************/
    /*************** FIN GESTIÓN VISUALIZACIÓN PUNTOS DE ABIERTOS Y CERRADOS********/
    /****************************************************************************/

    // visualizamos start. {}
    //visualizaCelda(marker_Open_publisher, markers_OpenList, cpstart.index);

    unsigned int explorados = 0;
    unsigned int currentIndex = cpstart.index;

    //ROS_INFO("SIZE: %d", openList.size());

    // Calcular tiempo que tarda el algoritmo
    ros::Time time_begin = ros::Time::now();

    // Si se ha encontrado un camino o no
    bool camino = false;

    // Nodo actual
    coupleOfCells COfCells;

    // Hasta no hallar un camino
    while(!openList.empty()) {

        // Sacar de abiertos, meter en cerrados, actualizar mapa.
        COfCells = openList.front();
        openList.pop_front();
        closedList.push_back(COfCells);
        setEstadoMapa(COfCells.index, CERRADO);

        //visualizaCelda(marker_Open_publisher, markers_OpenList, COfCells.index);

        // Es nodo objetivo, terminar
        if(COfCells.index == cpgoal.index) {
            camino = true;
            break;
        }

        // Obtener vecinos libres (expandir)
        expandirNodo(COfCells, cpstart, cpgoal);

        explorados++;
    }

    //ROS_INFO("Done!");

    // if the currentCell is the goalCell: success: path found
    if(camino) {
        // Tiempo que tardó el algoritmo
        ros::Time time_end = ros::Time::now();

        double secs = time_end.toSec() - time_begin.toSec();

        ROS_INFO("Tiempo = %f' s", secs);
        ROS_INFO("Distancia nodos:");

        //el plan lo construimos partiendo del goal, del parent del goal y saltando en cerrados "de parent en parent"
        //vamos insertando al final los waypoints (los nodos de cerrados), por tanto, cuando finaliza el bucle hay que darle la vuelta al plan
        //ROS_INFO("Se han explorado %u nodos y cerrados tiene %u nodos", explorados, (unsigned int)closedList.size());
        //ros::Duration(10).sleep();
        //convertimos goal a poseStamped nueva
        geometry_msgs::PoseStamped pose;
        pose.header.stamp =  ros::Time::now();
        pose.header.frame_id = goal.header.frame_id; //debe tener el mismo frame que el de la entrada
        pose.pose.position.x = goal_x;
        pose.pose.position.y = goal_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        //lo añadimos al plan
        plan.push_back(pose);

        visualizaCelda(marker_Open_publisher, markers_OpenList, costmap_->getIndex(mgoal_x, mgoal_y));

        coupleOfCells currentCouple = cpgoal;
        unsigned int currentParent = COfCells.parent;
        //ROS_INFO("Inserta en Plan GOAL: %f, %f PADRE: %u", pose.pose.position.x, pose.pose.position.y, currentParent);
        //ros::Duration(1).sleep();

        vector<coupleOfCells> nodosAQuitar;

        int ii=0;
        while (currentParent != cpstart.index) //e.d. mientras no lleguemos al nodo start
        {
            //encontramos la posición de currentParent en cerrados
            list<coupleOfCells>::iterator it=getPositionInList(closedList,currentParent);

            //hacemos esa posición que sea el currentCouple
            coupleOfCells currentCouple;
            currentCouple.index=currentParent;
            currentCouple.parent=(*it).parent;
            currentCouple.gCost=(*it).gCost;
            currentCouple.hCost=(*it).hCost;
            currentCouple.fCost=(*it).fCost;

            // Mostrar distnacia de nodos
            ROS_INFO("%f", currentCouple.fCost);

            //creamos una PoseStamped con la informaciuón de currentCouple.index

            //primero hay que convertir el currentCouple.index a world coordinates
            unsigned int mpose_x, mpose_y;
            double wpose_x, wpose_y;

            costmap_->indexToCells((*it).index, mpose_x, mpose_y);
            costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);

            //ROS_INFO("Las coordenadas de El PADRE de %u son (%u, %u) -> (%f, %f). Y su PADRE es %u.", currentParent, mpose_x,mpose_y,wpose_x, wpose_y, (*it).parent);
            //ros::Duration(1).sleep();

            //después creamos la pose
            geometry_msgs::PoseStamped pose;
            pose.header.stamp =  ros::Time::now();
            pose.header.frame_id = goal.header.frame_id; //debe tener el mismo frame que el de la entrada
            pose.pose.position.x = wpose_x;
            pose.pose.position.y = wpose_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            //insertamos la pose en el plan
            plan.push_back(pose);
            //ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);
            //hacemos que currentParent sea el parent de currentCouple
            currentParent = (*it).parent;

            visualizaCelda(marker_Open_publisher, markers_OpenList, currentParent);
            nodosAQuitar.push_back(*it);
        }

        ROS_INFO("EXPLORADOS: %d", explorados);

        //ROS_INFO("Sale del bucle de generación del plan.");
        std::reverse(plan.begin(),plan.end());

        // Pintar nodos cerrados
        list<coupleOfCells>::iterator itt=closedList.begin();
        while(itt != closedList.end()) {
            unsigned int mpose_x, mpose_y;
            double wpose_x, wpose_y;
            costmap_->indexToCells(itt->index, mpose_x, mpose_y);
            costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
            geometry_msgs::Point p;
            p.x = wpose_x;
            p.y = wpose_y;
            p.z = 0; //¿?

            // No agregar si esta en el camino
            bool estaEnElCamino = false;
            for(int i=0; i<nodosAQuitar.size() && !estaEnElCamino; i++) {
                if(nodosAQuitar[i].index == (*itt).index) {
                    estaEnElCamino = true;
                }
            }

            if(!estaEnElCamino)
                markers_ClosedList.points.push_back(p); //anyado el punto inicial

            itt++;
        }
        marker_Closed_publisher.publish(markers_ClosedList); //lo publico
        // Termina dibujar nodos cerrados

        //lo publica en el topic "planTotal"
        //publishPlan(plan);
        return true;
    }

    if(openList.empty()) // if the openList is empty: then failure to find a path
    {
        //ROS_INFO("Failure to find a path !");
        return false;
        //exit(1);
    }
};

void MyastarPlanner::inicializar() {

    // Tamano del mapa en celdas
    const int tama_x = costmap_->getSizeInCellsX();
    const int tama_y = costmap_->getSizeInCellsY();

    // Crear nuevo mapa
    m = new int*[tama_y];
    for(int i = 0; i < tama_y; ++i)
        m[i] = new int[tama_x];

    // Inicializar matriz
    for(int i=0; i<tama_y; i++)
        for(int j=0; j<tama_x; j++)
            m[i][j] = LIBRE;

    // Crear nuevo mapa de indices
    m_indices = new unsigned int*[tama_y];
    for(int i = 0; i < tama_y; ++i)
        m_indices[i] = new unsigned int[tama_x];

    // Inicializar matriz de indices
    for(int i=0; i<tama_y; i++)
        for(int j=0; j<tama_x; j++) {
            m_indices[i][j] = costmap_->getIndex((j/ANCHO)*ANCHO, (i/ANCHO)*ANCHO);
        }

    // Crear nuevo mapa de punteros
    m_punteros = new NodoJ*[tama_y];
    for(int i = 0; i < tama_y; ++i)
        m_punteros[i] = new NodoJ[tama_x];

    // Inicializar matriz de punteros
    for(int i=0; i<tama_y; i++)
        for(int j=0; j<tama_x; j++) {
            m_punteros[i][j].i = &m_indices[i][j];

            if(i-ANCHO >= 0)      m_punteros[i][j].u = &m_indices[i-ANCHO][j];
            if(i+ANCHO <  tama_y) m_punteros[i][j].d = &m_indices[i+ANCHO][j];
            if(j-ANCHO >= 0)      m_punteros[i][j].l = &m_indices[i][j-ANCHO];
            if(j+ANCHO <  tama_x) m_punteros[i][j].r = &m_indices[i][j+ANCHO];

            if(i-ANCHO >= 0      && i-ANCHO >= 0)      m_punteros[i][j].ul = &m_indices[i-ANCHO][j-ANCHO];
            if(i+ANCHO <  tama_y && i-ANCHO >= 0)      m_punteros[i][j].ur = &m_indices[i-ANCHO][j+ANCHO];
            if(j-ANCHO >= 0      && i+ANCHO <  tama_y) m_punteros[i][j].dl = &m_indices[i+ANCHO][j-ANCHO];
            if(j+ANCHO <  tama_x && i+ANCHO <  tama_y) m_punteros[i][j].dr = &m_indices[i+ANCHO][j+ANCHO];
        }

    // Buscar bloques juntos
    for(int ANCHO_IGNORAR=ANCHO_IGNORAR_MAX; ANCHO_IGNORAR>1; ANCHO_IGNORAR/=2)
    for(int i=ANCHO_IGNORAR; i<tama_y-ANCHO_IGNORAR*2; i+=ANCHO_IGNORAR) {
        for(int j=ANCHO_IGNORAR; j<tama_x-ANCHO_IGNORAR*2; j+=ANCHO_IGNORAR) {

            bool bloque = true;

            // Comprobar si en un area de bloque hay obstaculos.
            // En caso positivo marcar no anadir bloque
            for(int y=-1; y<ANCHO_IGNORAR+1 && bloque; y++) {
                for(int x=-1; x<ANCHO_IGNORAR+1 && bloque; x++) {
                    if(costmap_->getCost(j+x, i+y) >= OBSTACULO)
                        bloque = false;
                }
            }

            // Si se ha anadido un bloque, poner peso adecuado.
            bool bloque_anadido = false;

            // Si es un bloque valido para anadir, modificar indices
            if(bloque) {
                // Hacer que los indices vecinos del nodo central apunten a los nodos de afuera
                unsigned int indice = m_indices[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2];

                m_punteros[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2].u = &m_indices[i-1][j+ANCHO_IGNORAR/2];
                m_punteros[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2].d = &m_indices[i+ANCHO_IGNORAR+1][j+ANCHO_IGNORAR/2];
                m_punteros[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2].l = &m_indices[i+ANCHO_IGNORAR/2][j-1];
                m_punteros[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2].r = &m_indices[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR+1];

                m_punteros[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2].ul = &m_indices[i-1][j-1];
                m_punteros[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2].ur = &m_indices[i-1][j+ANCHO_IGNORAR+1];
                m_punteros[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2].dl = &m_indices[i+ANCHO_IGNORAR+1][j-1];
                m_punteros[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2].dr = &m_indices[i+ANCHO_IGNORAR+1][j+ANCHO_IGNORAR+1];

                // Hacer que los nodos del bloque apunten al central
                for(int y=0; y<ANCHO_IGNORAR; y++)
                    for(int x=0; x<ANCHO_IGNORAR; x++) {
                        m_indices[i+y][j+x] = indice;
                    }

                // Bloque anadido
                bloque_anadido = true;
            }
            // Si el bloque no esta libre de obstaculos, comprobar si horizontalmente, verticalmente o
            // en alguna diagonal esta libre de obstaculos, y hacer lo mismo de modificar los indices
            else {
                // Comprobar horizontalmente
                bool linea = true;

                for(int x=-1; x<ANCHO_IGNORAR+1 && linea; x++) {
                    if(costmap_->getCost(j+x, i) >= OBSTACULO)
                        linea = false;
                }

                if(linea) {
                    unsigned int indice = m_indices[i][j+ANCHO_IGNORAR/2];

                    m_punteros[i][j+ANCHO_IGNORAR/2].l = &m_indices[i][j-1];
                    m_punteros[i][j+ANCHO_IGNORAR/2].r = &m_indices[i][j+ANCHO_IGNORAR+1];

                    for(int x=0; x<ANCHO_IGNORAR; x++)
                        m_indices[i][j+x] = indice;

                    // Bloque anadido
                    bloque_anadido = true;
                }

                // Comprobar verticalmente
                linea = true;

                for(int y=-1; y<ANCHO_IGNORAR+1 && linea; y++) {
                    if(costmap_->getCost(j, i+y) >= OBSTACULO)
                        linea = false;
                }

                if(linea) {
                    unsigned int indice = m_indices[i+ANCHO_IGNORAR/2][j];

                    m_punteros[i+ANCHO_IGNORAR/2][j].l = &m_indices[i-1][j];
                    m_punteros[i+ANCHO_IGNORAR/2][j].r = &m_indices[i+ANCHO_IGNORAR+1][j];

                    for(int y=0; y<ANCHO_IGNORAR; y++)
                        m_indices[i+y][j] = indice;

                    // Bloque anadido
                    bloque_anadido = true;
                }

                // Comprobar primera diagonal
                linea = true;

                for(int x=-1; x<ANCHO_IGNORAR+1 && linea; x++)
                for(int y=-1; y<ANCHO_IGNORAR+1 && linea; y++) {
                    if(costmap_->getCost(j+x, i+y) >= OBSTACULO)
                        linea = false;
                }

                if(linea) {
                    unsigned int indice = m_indices[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2];

                    m_punteros[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2].ul = &m_indices[i-1][j-1];
                    m_punteros[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2].dr = &m_indices[i+ANCHO_IGNORAR+1][j+ANCHO_IGNORAR/2+1];

                    for(int x=0; x<ANCHO_IGNORAR; x++)
                    for(int y=0; y<ANCHO_IGNORAR; y++)
                        m_indices[i+y][j+x] = indice;

                    // Bloque anadido
                    bloque_anadido = true;
                }

                // Comprobar segunda diagonal
                linea = true;

                for(int x=ANCHO_IGNORAR; x>=-1 && linea; x--)
                for(int y=-1; y<ANCHO_IGNORAR+1 && linea; y++) {
                    if(costmap_->getCost(j+x, i+y) >= OBSTACULO)
                        linea = false;
                }

                if(linea) {
                    unsigned int indice = m_indices[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2];

                    m_punteros[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2].ur = &m_indices[i-1][j+ANCHO_IGNORAR+1];
                    m_punteros[i+ANCHO_IGNORAR/2][j+ANCHO_IGNORAR/2].dl = &m_indices[i+ANCHO_IGNORAR+1][j-1];

                    for(int x=ANCHO_IGNORAR; x>=-1; x--)
                    for(int y=0; y<ANCHO_IGNORAR; y++)
                        m_indices[i+y][j+x] = indice;

                    // Bloque anadido
                    bloque_anadido = true;
                }
            }
        }
    }
};

// Meter posibles nodos
void MyastarPlanner::expandirNodo(const coupleOfCells & COfCells, const coupleOfCells & cpstart, const coupleOfCells & cpgoal) {
    // Obtener vecinos libres (expandir)
    vector <unsigned int> neighborCells=findFreeNeighborCell(COfCells.index);

    // Ignorar nodos cerrados
    vector <unsigned int> neighborNotInClosedList;
    for(uint i=0; i<neighborCells.size(); i++)
    {
        // Si no esta en cerrados, meter en lsta de no cerrados
        if(getEstadoMapa(neighborCells[i]) != CERRADO)
            neighborNotInClosedList.push_back(neighborCells[i]);
    }

    // Meter posibles nodos
    vector <unsigned int> neighborsNotInOpenList;
    for(uint i=0; i<neighborNotInClosedList.size(); i++)
    {
        // Solo meter nodos libres
        if(getEstadoMapa(neighborNotInClosedList[i]) != ABIERTO) {

            if(esObstaculo(neighborNotInClosedList[i]))
                addNeighborCellToClosedList(closedList, neighborNotInClosedList[i], COfCells.index);
            else {
                addNeighborCellToOpenList(openList, neighborNotInClosedList[i], COfCells.index, COfCells.gCost, cpstart.index, cpgoal.index);
                neighborsNotInOpenList.push_back(neighborNotInClosedList[i]);
            }
        }
    }
}

//calculamos H como la distancia euclídea hasta el goal
double MyastarPlanner::calculateHCost(unsigned int start, unsigned int goal) {
    unsigned int mstart_x, mstart_y, mgoal_x, mgoal_y;
    double wstart_x, wstart_y, wgoal_x, wgoal_y;

    //trasformamos el indice de celdas a coordenadas del mundo.
    //ver http://docs.ros.org/indigo/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html

    costmap_->indexToCells(start, mstart_x, mstart_y);
    costmap_->mapToWorld(mstart_x, mstart_y, wstart_x, wstart_y);
    costmap_->indexToCells(goal, mgoal_x, mgoal_y);
    costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);

    return sqrt((pow(wstart_x - wgoal_x,2))+pow(wstart_y - wgoal_y, 2));
}


//comparamos F para dos nodos.
bool MyastarPlanner::compareFCost(coupleOfCells const &c1, coupleOfCells const &c2)
{
    return c1.fCost < c2.fCost;
}

/*******************************************************************************/
//Function Name: getPositnionInList
//Inputs:the cellID, the list
//Output: index of the cell in the list
//Description: it is used to search the index of a cell in a list
/*********************************************************************************/
list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID)
{
    for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++) {
        if (it->index == cellID)
            return it;
    }
}

/*******************************************************************************
* Function Name: findFreeNeighborCell
* Inputs: the row and columun of the current Cell
* Output: a vector of free neighbor cells of the current cell
* Description:it is used to find the free neighbors Cells of a the current Cell in the grid
* Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/
vector <unsigned int> MyastarPlanner::findFreeNeighborCell (unsigned int CellID) {

    unsigned int mx, my, mx_v, my_v;

    costmap_->indexToCells(CellID,mx,my);
    //ROS_INFO("Viendo vecinos de index: %u, Map coords: (%u,%u)", CellID, mx,my);

    vector <unsigned int> freeNeighborCells;

    if(*m_punteros[my][mx].u != -1) {
        costmap_->indexToCells(*m_punteros[my][mx].u, mx_v, my_v);

        if(costmap_->getCost(mx_v, my_v) < OBSTACULO)
            freeNeighborCells.push_back(*m_punteros[my][mx].u);
    }

    if(*m_punteros[my][mx].d != -1) {
        costmap_->indexToCells(*m_punteros[my][mx].d, mx_v, my_v);

        if(costmap_->getCost(mx_v, my_v) < OBSTACULO)
            freeNeighborCells.push_back(*m_punteros[my][mx].d);
    }

    if(*m_punteros[my][mx].l != -1) {
        costmap_->indexToCells(*m_punteros[my][mx].l, mx_v, my_v);

        if(costmap_->getCost(mx_v, my_v) < OBSTACULO)
            freeNeighborCells.push_back(*m_punteros[my][mx].l);
    }

    if(*m_punteros[my][mx].r != -1) {
        costmap_->indexToCells(*m_punteros[my][mx].r, mx_v, my_v);

        if(costmap_->getCost(mx_v, my_v) < OBSTACULO)
            freeNeighborCells.push_back(*m_punteros[my][mx].r);
    }

    if(*m_punteros[my][mx].ul != -1) {
        costmap_->indexToCells(*m_punteros[my][mx].ul, mx_v, my_v);

        if(costmap_->getCost(mx_v, my_v) < OBSTACULO)
            freeNeighborCells.push_back(*m_punteros[my][mx].ul);
    }

    if(*m_punteros[my][mx].ur != -1) {
        costmap_->indexToCells(*m_punteros[my][mx].ur, mx_v, my_v);

        if(costmap_->getCost(mx_v, my_v) < OBSTACULO)
            freeNeighborCells.push_back(*m_punteros[my][mx].ur);
    }

    if(*m_punteros[my][mx].dl != -1) {
        costmap_->indexToCells(*m_punteros[my][mx].dl, mx_v, my_v);

        if(costmap_->getCost(mx_v, my_v) < OBSTACULO)
            freeNeighborCells.push_back(*m_punteros[my][mx].dl);
    }

    if(*m_punteros[my][mx].dr != -1) {
        costmap_->indexToCells(*m_punteros[my][mx].dr, mx_v, my_v);

        if(costmap_->getCost(mx_v, my_v) < OBSTACULO)
            freeNeighborCells.push_back(*m_punteros[my][mx].dr);
    }

    return freeNeighborCells;
}

/*******************************************************************************/
//Function Name: isContains
//Inputs: the list, the cellID
//Output: true or false
//Description: it is used to check if a cell exists in the open list or in the closed list
/*********************************************************************************/
bool isContains(list<coupleOfCells> & list1, int cellID)
{
    for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++) {
        if (it->index == cellID)
            return true;
    }
    return false;
}

double MyastarPlanner::getMoveCost(unsigned int here, unsigned int there) {
    //calculo el coste de moverme entre celdas adyacentes como la distancia euclídea.
    return calculateHCost(here,there);
}

/*******************************************************************************/
//Function Name: addNeighborCellsToOpenList
//Inputs: the open list, the neighbors Cells and the parent Cell
//Output:
//Description: it is used to add the neighbor Cells to the open list
/*********************************************************************************/
void MyastarPlanner::addNeighborCellToOpenList(Lista & OPL, unsigned int neighborCell, unsigned int parent, float gCostParent, unsigned int startCell, unsigned int goalCell) //,float tBreak)
{
    vector <coupleOfCells> neighborsCellsOrdered;

    // Metemos los nodos en la lista
    coupleOfCells CP;
    CP.index=neighborCell; //insert the neighbor cell
    CP.parent=parent; //insert the parent cell

    // Calculamos el coste desde este nodo hasta el objetivo
    CP.gCost = gCostParent + getMoveCost(parent, CP.index);
    CP.hCost = calculateHCost(CP.index, goalCell) * (1 + calculateHCost(CP.index, goalCell)/calculateHCost(startCell, goalCell));
    CP.fCost = CP.gCost + CP.hCost;

    // Metemos celda
    OPL.push(CP);

    // Actualizamos mapa
    setEstadoMapa(neighborCell, ABIERTO);
}

// Mete un nodo en cerrados
void MyastarPlanner::addNeighborCellToClosedList(list<coupleOfCells> & OPL, unsigned int neighborCell, unsigned int parent) //,float tBreak)
{
    // Metemos en la lista
    coupleOfCells CP;
    CP.index=neighborCell; //insert the neighbor cell
    CP.parent=parent; //insert the parent cell
    OPL.push_back(CP);

    // Actualizamos mapa
    setEstadoMapa(neighborCell, CERRADO);
}

// Dado un indice devuelve la casilla de la matriz
int MyastarPlanner::getEstadoMapa(unsigned int indice) {
    // Obtener coordenadas de matriz segun indice
    unsigned int mx, my;
    costmap_->indexToCells(indice,mx,my);

    // Devolver casilla de la matriz
    return m[my][mx];
}

// Cambia el estado de la matriz segun un indice
void MyastarPlanner::setEstadoMapa(unsigned int indice, int estado) {
    // Obtener coordenadas de matriz segun indice
    unsigned int mx, my;
    costmap_->indexToCells(indice,mx,my);

    // Actualizar estado
    m[my][mx] = estado;
}

/********VISUALIZAR ESPACIO DE BUSQUEDA *************************/

void MyastarPlanner::inicializaMarkersPoints(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
    marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
    marker.header.stamp =  ros::Time::now();
    marker.ns = ns;

    marker.action = visualization_msgs::Marker::ADD; //la otra es DELETE
    marker.pose.orientation.w = 0.0;
    marker.id = id;
    marker.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    marker.scale.x = costmap_->getResolution()*3;
    marker.scale.y = costmap_->getResolution()*3;

    // Points are green
    marker.color.g = g;
    marker.color.r = r;
    marker.color.b = b;
    marker.color.a = 1.0;
}

void MyastarPlanner::inicializaMarkersLine_List(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
    marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
    marker.header.stamp =  ros::Time::now();
    marker.ns = ns;

    marker.action = visualization_msgs::Marker::ADD; //la otra es DELETE
    marker.pose.orientation.w = 0.0;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;

    //Line lists also have some special handling for scale: only scale.x is used and it controls the width of the line segments.
    marker.scale.x = marker.scale.y = 0.5;
    // marker.scale.y = costmap_->getResolution();

    // Points are green
    marker.color.g = g;
    marker.color.r = r;
    marker.color.b = b;
    marker.color.a = 1.0;

}

void MyastarPlanner::visualizaCoords(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y) {
    //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0; //¿?

    marker.points.push_back(p); //anyado el punto inicial
    where.publish(marker); //lo publico
    //points.points.pop_back(); //quito el punto de la lista de puntos, lo borro con DELETE cuando lo saque de abiertos.
}

void MyastarPlanner::visualizaCoordsLineUp(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y, double z) {
    //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    where.publish(marker); //lo publico
    //points.points.pop_back(); //quito el punto de la lista de puntos, lo borro con DELETE cuando lo saque de abiertos.
}

void MyastarPlanner::visualizaCelda(ros::Publisher where, visualization_msgs::Marker &marker, unsigned int index) {
    unsigned int mpose_x, mpose_y;
    double wpose_x, wpose_y;
    costmap_->indexToCells(index, mpose_x, mpose_y);
    costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
    visualizaCoords(where, marker, wpose_x, wpose_y);

}

void MyastarPlanner::visualizaLista(ros::Publisher where, visualization_msgs::Marker &marker, vector<unsigned int> lista) {
    for(vector<unsigned int>::iterator i = lista.begin(); i != lista.end(); ++i)
    {
        unsigned int mpose_x, mpose_y;
        double wpose_x, wpose_y;
        costmap_->indexToCells(*i, mpose_x, mpose_y);
        costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
        //PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
        geometry_msgs::Point p;
        p.x = wpose_x;
        p.y = wpose_y;
        p.z = 0; //¿?

        marker.points.push_back(p);
    }
    where.publish(marker);
    //quitar neighborCells de points .popback
}

void MyastarPlanner::limpiaMarkers(ros::Publisher where, visualization_msgs::Marker &marker) {
    if (!marker.points.empty()) {
        marker.action = visualization_msgs::Marker::DELETE;
        where.publish(marker);
        marker.action = visualization_msgs::Marker::ADD;
    }
    marker.points.clear();
}


///////////// CLASE LISTA ///////////

// Se copia a un objeto los datos de otro
void Lista::copiarCells(coupleOfCells & p, const coupleOfCells & q) {
    p.index  = q.index;
    p.parent = q.parent;
    p.gCost  = q.gCost;
    p.hCost  = q.hCost;
    p.fCost  = q.fCost;
};

// Devuelve verdadero si el primero es mayor que el segundo
bool Lista::orden(const coupleOfCells & p, const coupleOfCells & q) {
    return (p.fCost < q.fCost);
}

Lista::Lista() {
    tama    = 0;
    primero = 0;
    ultimo  = 0;
    limite  = 10000;
};

void Lista::setLimite(int l) {
    limite = l;
};

void Lista::push(const coupleOfCells & c) {
    // Si la lista esta vacia, es el primero y el ultimo
    if(tama == 0) {

        // Crear nuevo y copiar datos
        primero = new Nodo;
        primero->sig = 0;
        primero->ant = 0;
        copiarCells(primero->c, c);

        ultimo = primero;
    }
    // En caso contrario recorrer e insertar ordenadamente segun f
    else {
        // Recorrer y meter en orden
        Nodo * p = primero;
        Nodo * a = primero; // Nodo anterior

        // Recorrer hasta llegar a un nodo siguiente = 0 (no existe)
        // O hasta encontrar el orden
        while(p != 0 && orden(p->c, c)) {
            a = p;
            p = p->sig; // Avanzar puntero
        }

        // Si es el ultimo, crear nuevo al final
        if(p == 0) {
            a->sig = new Nodo;
            a->sig->sig = 0;
            copiarCells(a->sig->c, c);
            ultimo = a->sig;
            ultimo->ant = a;
            ultimo->sig = 0;
        }
        // Si es el primero,
        else if(p == primero) {
            Nodo * aux = primero;
            primero = new Nodo;
            primero->ant = 0;
            copiarCells(primero->c, c);
            primero->sig = aux;
            primero->sig->ant = primero;
        }
        // Si esta en el medio
        else {
            // Crear nuevo siguiente al nodo anterior
            p->ant->sig = new Nodo;
            copiarCells(p->ant->sig->c, c);
            p->ant->sig->sig = p;
            p->ant->sig->ant = p->ant;
            p->ant = p->ant->sig;
        }
    }

    // Aumentar tamano
    tama++;

    // Limitar
    if(tama > limite) {
        pop_back();
    }
};

void Lista::pop_front() {
    // Si es tama 1, quitar el que queda
    if(tama == 1) {
        delete primero;
        primero = 0;
        ultimo = 0;
    }
    else {
        // Guardar el primero original
        Nodo * sig = primero;

        // Avanzar primero
        primero = primero->sig;

        // Anterior del primero apunta a 0
        primero->ant = 0;

        // Borrar viejo (si existe)
        delete sig;
    }

    // Reducir tamano
    tama--;
};

void Lista::pop_back() {
    // Si es tama 1, quitar el que queda
    if(tama == 1) {
        delete ultimo;
        primero = 0;
        ultimo = 0;
    }
    else {
        // Guardar el ultimo original
        Nodo * ant = ultimo;

        // Retroceder ultimo
        ultimo = ultimo->ant;

        // Siguiente del ultimo apunta a 0
        ultimo->sig = 0;

        // Borrar viejo
        delete ant;
    }

    // Reducir tamano
    tama--;
};

bool Lista::empty() {
    return tama == 0;
};

int Lista::size() {
    return tama;
};

coupleOfCells Lista::front() {
    return primero->c;
};

coupleOfCells Lista::back() {
    return ultimo->c;
};

bool Lista::isContains(int cellID) {
    // Recorrer
    Nodo * p = primero;

    // Recorrer hasta llegar a un nodo siguiente = 0 (no existe)
    while(p != 0) {
        if (p->c.index == cellID)
            return true;

        p = p->sig; // Avanzar puntero
    }

    return false;
};

Nodo * Lista::getPositionInList(unsigned int cellID) {
    // Recorrer
    Nodo * p = primero;

    // Recorrer hasta llegar a un nodo siguiente = 0 (no existe)
    int i=0;
    while(p != 0) {
        if (p->c.index == cellID)
            return p;

        p = p->sig; // Avanzar puntero
    }
};

void Lista::clear() {
    // Recorrer
    Nodo * p = primero;

    // Recorrer hasta llegar a un nodo siguiente = 0 (no existe)
    // O hasta encontrar el orden
    while(p != 0) {
        p = p->sig; // Avanzar puntero

        if(p != 0)
            delete p->ant;
    }

    delete ultimo;
    tama = 0;
    primero = 0;
    ultimo  = 0;
};

void Lista::print() {
    ROS_INFO("---------LISTA---------");
    // Recorrer
    Nodo * p = primero;

    // Recorrer hasta llegar a un nodo siguiente = 0 (no existe)
    // O hasta encontrar el orden
    while(p != 0) {
        ROS_INFO("%f", p->c.fCost);
        p = p->sig;
    }
    ROS_INFO("-----------------------");
};

}
