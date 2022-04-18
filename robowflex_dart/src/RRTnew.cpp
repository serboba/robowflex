/*
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

/* Author: Ioan Sucan */

//
// Created by serboba on 22.01.22.
//

#include <robowflex_dart/RRTnew.h>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/String.h"

ompl::geometric::RRTnew::RRTnew(const base::SpaceInformationPtr &si, std::vector<std::vector<int>> gr_indices,
                                bool useIsolation, int goalIndex)
        : base::Planner(si, "RRTnew")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;
    group_indices = gr_indices;

    Planner::declareParam<double>("range", this, &RRTnew::setRange, &RRTnew::getRange, "0.:1.:10000.");
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    useIsolation_ = useIsolation;
    goalIndex_ = goalIndex;
    addPlannerProgressProperty("best cost DOUBLE", [this] { return bestCostProgressProperty(); });
    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
}

ompl::geometric::RRTnew::~RRTnew()
{
    freeMemory();
}

void ompl::geometric::RRTnew::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    opt_ = std::make_shared<ompl::base::IsoManipulationOptimization>(si_,group_indices);
    pdef_->setOptimizationObjective(opt_);

    bestCost_ = opt_->infiniteCost();
    incCost = opt_->infiniteCost();
}

void ompl::geometric::RRTnew::freeMemory()
{
    std::vector<Motion *> motions;
    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr){}
            si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }



}

void ompl::geometric::RRTnew::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();

    iterations_ = 0;
    bestCost_ = base::Cost(INFINITY);
}
void ompl::geometric::RRTnew::getChangedIndices(const base::State* rfrom, const base::State* rto,
                                                std::vector<int> &indices_) const{


    std::vector<double> s1,s2;
    si_->getStateSpace()->copyToReals(s1,rfrom);
    si_->getStateSpace()->copyToReals(s2,rto);

    for (size_t i= 0; i < s1.size() ; i++) {
        if (abs(s1.at(i) - s2.at(i)) > 0.0001) {
            indices_.push_back(i);
        }
    }

}



void ompl::geometric::RRTnew::buildIsoStates(const std::vector<double> &from_,const std::vector<double> &to_,
                                                                         std::vector<int> &changed_index_groups,
                                                                         std::vector<ompl::base::State* > &iso_ ) {
    std::vector<double> intermediate_st(from_.size());
    intermediate_st = from_;


    for (size_t v = 0; v < changed_index_groups.size(); v++) {
        for (int i: group_indices.at(changed_index_groups.at(v))) {
            intermediate_st[i] = to_[i];
        }
        base::State *temp = si_->getStateSpace()->allocState(); // todo mem leak?
        si_->getStateSpace()->copyFromReals(temp, intermediate_st);
        iso_.push_back(temp);
    }
}

void ompl::geometric::RRTnew::buildIsoStates(ompl::base::State * from_, const ompl::base::State *to,
                                             std::vector<int> &changed_index_groups,
                                             std::vector<ompl::base::State* > &iso_ )
{
    std::vector<double> intermediate_st;
    std::vector<double> to_;
    base::State * it_ = from_;
    si_->getStateSpace()->copyToReals(intermediate_st,from_);
    si_->getStateSpace()->copyToReals(to_,to);
    for (size_t v = 0; v < changed_index_groups.size(); v++) {
        for (int i: group_indices.at(changed_index_groups.at(v))) {
            intermediate_st[i] = to_[i];
        }
        ompl::base::State * newState = si_->getStateSpace()->allocState();
        si_->getStateSpace()->copyFromReals(newState,intermediate_st);

        if (si_->checkMotion(it_,newState))
            iso_.push_back(newState);
        else{
            freeStates(iso_);
            si_->freeState(newState);
            return;
        }
        it_ = newState;
    }

}


std::vector<int> ompl::geometric::RRTnew::getChangedGroups(const std::vector<double> &from_,const std::vector<double> &to_){
    std::vector<int> groups;
    for(size_t i = 0 ; i < group_indices.size(); i++){
        for(auto index : group_indices.at(i)){
            if(abs(from_.at(index)-to_.at(index))  > 0.0001){
                groups.push_back(i);
                break;
            }
        }
    }
    return groups;
}

std::vector<int> ompl::geometric::RRTnew::getChangedGroups(const base::State* rfrom, const base::State* rto){

    std::vector<double> from_,to_;
    si_->getStateSpace()->copyToReals(from_,rfrom);
    si_->getStateSpace()->copyToReals(to_,rto);

    std::vector<int> groups;
    for(size_t i = 0 ; i < group_indices.size(); i++){
        for(auto index : group_indices.at(i)){
            if(abs(from_.at(index)-to_.at(index))  > 1e-10){
                groups.push_back(i);
                break;
            }
        }
    }

    return groups;
}


void ompl::geometric::RRTnew::isolateStates(const base::State* rfrom, const base::State* rto, std::vector<ompl::base::State*> &iso_){

    std::vector<double> from_,to_;
    si_->getStateSpace()->copyToReals(from_,rfrom);
    si_->getStateSpace()->copyToReals(to_,rto);

    std::vector<int> groups = getChangedGroups(from_,to_);

    //std::vector<int> changed_index_groups = reorderGroup(groups, prev_index);

    buildIsoStates(from_,to_,groups,iso_);

}

void ompl::geometric::RRTnew::createNewMotion(const base::State *st, ompl::geometric::RRTnew::Motion *premotion,
                                              ompl::geometric::RRTnew::Motion *newmotion){
    //auto *motion = new Motion(si_);
    si_->copyState(newmotion->state, st);

    newmotion->parent = premotion;
    newmotion->root = premotion->root;
    newmotion->cost = opt_->motionCost(premotion->state,st);
    newmotion->index_changed = getChangedIndex(premotion->state,st);

}


bool ompl::geometric::RRTnew::validMotionCheck(const bool start, const base::State *from_,const base::State *to_){
    bool validmo= start ? si_->checkMotion(from_, to_) :
                  si_->isValid(to_) && si_->checkMotion(to_, from_);

    return validmo;

}

void ompl::geometric::RRTnew::getMotionVectors(Motion * mot_,std::vector<Motion *> &vec)
{
    /* construct the motion vec */
    Motion *solution = mot_;
    while (solution != nullptr)
    {
        vec.push_back(solution);
        solution = solution->parent;
    }

}

void ompl::geometric::RRTnew::getIntermediateState(const ompl::base::State *from,const ompl::base::State * to, ompl::base::State *state, int index_group)
{
    std::vector<double> s_new_v,to_;

    si_->getStateSpace()->copyToReals(s_new_v,from);
    si_->getStateSpace()->copyToReals(to_,to);

    std::vector<int> group = group_indices.at(index_group);
    for(size_t i = 0; i< group.size(); i++){
        s_new_v[group.at(i)] = to_[group.at(i)];
    }

    si_->getStateSpace()->copyFromReals(state,s_new_v);
}


int ompl::geometric::RRTnew::getChangedIndex(const ompl::base::State *from,const ompl::base::State * to){

    // setze voraus dass cost 1 zw. from und to, keine equal states

    std::vector<double> s_from,s_to;
    si_->getStateSpace()->copyToReals(s_from,from);
    si_->getStateSpace()->copyToReals(s_to,to);

    for(size_t i = 0; i < group_indices.size() ; i++)
    {
        for(auto const &index_in_gr : group_indices.at(i))
        {
            if(abs(s_from.at(index_in_gr) - s_to.at(index_in_gr)) > 1e-10)
                return i;
        }
    }
    OMPL_ERROR("NOTHING FOUND");
    std::cout<< "nothing found" << std::endl;
    si_->printState(from);
    si_->printState(to);
    return -1;
}


std::vector<ompl::base::State * > ompl::geometric::RRTnew::getStates(std::vector<ompl::geometric::RRTnew::Motion *> motions)
{
    std::vector<ompl::base::State * > states;
    states.reserve(motions.size());

    for(auto &i: motions){
        if(!states.empty() && si_->equalStates(i->state,states.back())){
            // std::cout << "----STATES EQUAL :" << std::endl;
            continue;
        }
        states.push_back(i->state);
    }

    return states;
}


void ompl::geometric::RRTnew::freeStates(std::vector<ompl::base::State*> &states)
{
    for(auto &st : states)
        si_->freeState(st);
    states.clear();
}

void ompl::geometric::RRTnew::freeStates(std::vector<std::pair<ompl::base::State *,int >> &states)
{
    for(auto &st : states)
        si_->freeState(st.first);
    states.clear();
}

bool ompl::geometric::RRTnew::reConnect(ompl::base::State *from,
                                        std::vector<std::pair<ompl::base::State *,int >> &queue_,
                                        std::vector<ompl::base::State * > &rewireResult )
{
    for(size_t i = 0; i < queue_.size(); i++)
    {
        ompl::base::State * newState = si_->allocState(); // todo mem leak
        getIntermediateState(from,queue_.at(i).first,newState,queue_.at(i).second);
        if(!si_->checkMotion(from,newState))
        {
            si_->freeState(newState);
            return false;
        }

        rewireResult.push_back(newState);
        from = newState;
    }
    return true;
}


void ompl::geometric::RRTnew::reConnect(ompl::base::State *from, std::vector<std::pair<ompl::base::State *,int >> &prio_,
                                                                     std::vector<std::pair<ompl::base::State *,int >> &stack_,
                                                                     std::vector<ompl::base::State *> &rewireResult) {

    if (reConnect(from, prio_, rewireResult)) {
        ompl::base::State * temp_from = si_->getStateSpace()->allocState();
        si_->getStateSpace()->copyState(temp_from,rewireResult.back());

        if (reConnect(temp_from, stack_, rewireResult)) { // reconnection successful
            si_->freeState(temp_from);
        }
        else{ // collision during reconnection
            si_->freeState(temp_from);
            freeStates(rewireResult);
        }
    }else{ // collision during first reconnection
        freeStates(rewireResult);
    }

    // all states newly allocated delete old
    freeStates(prio_);
    freeStates(stack_);
}

void ompl::geometric::RRTnew::findNextFragment(int start_index, int prev_index, std::vector<ompl::base::State *> &mainPath, bool sameFragmentType,
                                              std::vector<std::pair<ompl::base::State *, int>> &foundFragment)
{
    if(sameFragmentType)
    {
        for (size_t i = start_index; i < mainPath.size() - 1; i++) {
            if (getChangedIndex(mainPath.at(i - 1), mainPath.at(i)) == prev_index) {

                ompl::base::State *xstate = si_->getStateSpace()->allocState();
                si_->getStateSpace()->copyState(xstate, mainPath.at(i));
                foundFragment.push_back(std::make_pair(xstate, getChangedIndex(mainPath.at(i - 1), mainPath.at(i))));
            }
            else
                return;
        }
    }
    else
    {
        for (size_t i = start_index; i < mainPath.size() - 1; i++) {
            if (getChangedIndex(mainPath.at(i - 1), mainPath.at(i)) != prev_index) {

                ompl::base::State *xstate = si_->getStateSpace()->allocState();
                si_->getStateSpace()->copyState(xstate, mainPath.at(i));
                foundFragment.push_back(std::make_pair(xstate, getChangedIndex(mainPath.at(i - 1), mainPath.at(i))));
            }
            else
                return;
        }
    }

}

int ompl::geometric::RRTnew::pathDefrag(std::vector<ompl::base::State *> &mainPath) {

    int rewireCount = 0;
    int prev_index = getChangedIndex(mainPath.at(0),mainPath.at(1));

    for(size_t toID = 1; toID < mainPath.size()-1; ++toID){

        size_t fromID = toID-1;

        ompl::base::State *fromState = mainPath.at(fromID);
        ompl::base::State  *toState = mainPath.at(toID);

        if(getChangedIndex(fromState,toState) != prev_index)
        {
            size_t subsearch_id = toID;
            std::vector<std::pair<ompl::base::State *,int>> merge_fragment;
            std::vector<std::pair<ompl::base::State *,int>> push_fragment;

            findNextFragment(subsearch_id,prev_index,mainPath,false,push_fragment);

            if(!push_fragment.empty())
                subsearch_id += push_fragment.size();
            else{
                prev_index = getChangedIndex(mainPath.at(toID-1),mainPath.at(toID));
                continue;
            }

            findNextFragment(subsearch_id,prev_index,mainPath, true, merge_fragment);

            if(merge_fragment.empty()){
                freeStates(push_fragment);

                prev_index = getChangedIndex(mainPath.at(toID-1),mainPath.at(toID));
                continue;
            }

            if(subsearch_id >= mainPath.size())
                OMPL_ERROR("SUB SEARCH ID OVER SIZE ERR"); // should never occur

            std::vector<ompl::base::State * > rewiredConnection;

            reConnect(fromState,merge_fragment,push_fragment,rewiredConnection);

            if(rewiredConnection.empty()){ // no rewiring possible because of invalid motion but no problem, just start with next the next state and go on
                prev_index = getChangedIndex(mainPath.at(toID-1),mainPath.at(toID));
                continue;
            }

            rewireCount+=rewiredConnection.size();
            int l = 0;


            for(size_t j = toID; j < toID+rewiredConnection.size(); j++)
                si_->freeState(mainPath.at(j));

            int end_index = mainPath.size()-toID  -rewiredConnection.size();
            mainPath.erase(mainPath.begin()+toID, mainPath.end()-end_index);

            for(size_t j = toID; j <toID+rewiredConnection.size(); j++)
            {
                mainPath.insert(mainPath.begin()+j,rewiredConnection.at(l));
                l++;
            }

            fromID = fromID+merge_fragment.size(); //
            if(toID>= mainPath.size()-1)
                return rewireCount;
            prev_index = getChangedIndex(mainPath.at(fromID),mainPath.at(fromID+1));
            toID = fromID+1;
        }

    }
    return rewireCount;

}

int ompl::geometric::RRTnew::getCostPath(std::vector<ompl::base::State * > &states_)
{

    if(states_.size()<2)
        return 1;
    int prev_index = getChangedIndex(states_.at(0),states_.at(1));
    int cost = 1;
    for (size_t i = 1; i < states_.size()-1 ; ++i) {

        if(prev_index != getChangedIndex(states_.at(i),states_.at(i+1)))
        {
            cost++;
            prev_index = getChangedIndex(states_.at(i),states_.at(i+1));
        }
    }

    return cost;
}

void ompl::geometric::RRTnew::getFragmentIDs(std::vector<ompl::base::State*> &path,std::vector<Fragment> &fragmentIDs, bool goalFragment = false)
{
    fragmentIDs.clear();
    int prev_index = getChangedIndex(path.at(0),path.at(1));

    for(size_t i = 1; i < path.size()-1; i++)
    {
        if(prev_index != getChangedIndex(path.at(i),path.at(i+1)))
        {
            // check for equal states, that may occur if we skip fragments
            if(si_->equalStates(path.at(i),path.at(i+1))){
                std::cout<<"fragment equal states!! fixed!" << std::endl;
                path.erase(path.begin()+(i+1));
                continue;
            }
            if(fragmentIDs.size()>0)
                    fragmentIDs.push_back(Fragment(fragmentIDs.back().end_index+1,i, prev_index));
                else
                    fragmentIDs.push_back(Fragment(0,i, prev_index));

            prev_index = getChangedIndex(path.at(i),path.at(i+1));
        }
    }

    if(prev_index != fragmentIDs.back().id)
        fragmentIDs.push_back(Fragment(fragmentIDs.back().end_index+1,path.size()-1, prev_index));

    if(goalFragment)
    {
        std::vector<Fragment> goalfrags;
        for(size_t i = 0; i < fragmentIDs.size(); i++)
        {
            if(fragmentIDs.at(i).id == goalIndex_)
                goalfrags.push_back(fragmentIDs.at(i));
        }
        fragmentIDs = goalfrags;
    }
}

void ompl::geometric::RRTnew::getFragment(int start_index, int end_index,
                                          std::vector<ompl::base::State*> &mainPath, std::vector<std::pair<ompl::base::State *,int>> &fragment)
{
    for(int j = start_index; j< end_index; j++){
        ompl::base::State * temp = si_->allocState();
        si_->getStateSpace()->copyState(temp,mainPath.at(j));
        fragment.push_back(std::make_pair(temp, getChangedIndex(mainPath.at(j-1),mainPath.at(j))));
    }
}

void ompl::geometric::RRTnew::shortcutToNextGoalFragment(std::vector<ompl::base::State*> &mainPath)
{
    std::vector<Fragment> fragmentIDs;
    getFragmentIDs(mainPath,fragmentIDs,true);

    if(fragmentIDs.size()<2)
        return;
    std::vector<std::pair<ompl::base::State *,int>> subpathA;
    std::vector<std::pair<ompl::base::State *,int>> subpathB;
    std::vector<ompl::base::State *> result_;

    getFragment(fragmentIDs.at(0).start_index+1,fragmentIDs.at(0).end_index+1,mainPath,subpathA);
    getFragment(fragmentIDs.at(0).end_index+1,fragmentIDs.at(1).start_index,mainPath,subpathB);

    ompl::base::State * temp_from = si_->getStateSpace()->allocState();
    std::vector<double> tx(si_->getStateDimension(),0.0);
    si_->getStateSpace()->copyFromReals(temp_from,tx);

    reConnect(temp_from,subpathB,subpathA,result_);

    if(result_.empty()){
        si_->freeState(temp_from);
        return;
    }

    ompl::base::State * start_state = si_->allocState();
    si_->getStateSpace()->copyState(start_state,result_.at(0));
    subpathA.push_back(std::make_pair(start_state, getChangedIndex(mainPath.at(0),result_.at(0))));

    getFragment(1,result_.size(),result_,subpathA);
    getFragment(fragmentIDs.at(1).start_index,mainPath.size(),mainPath,subpathB);
    freeStates(result_);

    reConnect(temp_from,subpathA,subpathB,result_);

    if(!result_.empty())
    {
        freeStates(mainPath);
        mainPath = result_;
        mainPath.insert(mainPath.begin(),temp_from);
        return;
    }
    else
        si_->freeState(temp_from);

}


void ompl::geometric::RRTnew::trySkipFragment(std::vector<ompl::base::State*> &mainPath)
{
    std::vector<Fragment> fragmentIDs;
    getFragmentIDs(mainPath,fragmentIDs);

    for(size_t i = 0;  i < fragmentIDs.size()-1;  i++)
    {
        std::vector<std::pair<ompl::base::State *,int>> pathA;
        std::vector<std::pair<ompl::base::State *,int>> pathB;
        std::vector<ompl::base::State *> result_;


        ompl::base::State *temp_from = si_->getStateSpace()->allocState();
        std::vector<double> tx(si_->getStateDimension(), 0.0);
        si_->getStateSpace()->copyFromReals(temp_from, tx);

        if(i != 0){
            getFragment(1, fragmentIDs.at(i).start_index, mainPath, pathA);
            getFragment(fragmentIDs.at(i).end_index + 1, mainPath.size(), mainPath, pathB);

            if (pathA.size() < 1 || pathB.size() < 1) {
                freeStates(pathA);
                freeStates(pathB);
                si_->freeState(temp_from);
                continue;
            }
            reConnect(temp_from, pathA, pathB, result_);

        }
        else
        {
            getFragment(fragmentIDs.at(i).end_index+1,mainPath.size(),mainPath,pathB);
            if(pathB.size()<1){
                si_->freeState(temp_from);
                continue;
            }
            pathB.front().second = getChangedIndex(mainPath.at(fragmentIDs.at(i).end_index),pathB.front().first);

            if(!reConnect(temp_from,pathB,result_)){ // reconnect only one path segment directly to the state
                freeStates(pathB);
                freeStates(result_);
                si_->freeState(temp_from);
                continue;
            }

        }

        if(!result_.empty())
        {
            freeStates(mainPath);

            mainPath = result_;
            mainPath.insert(mainPath.begin(),temp_from);
            getFragmentIDs(mainPath,fragmentIDs);

        }
        else
            si_->freeState(temp_from);
        // no reconnection / jump over possible
    }



}
void ompl::geometric::RRTnew::simplifyActionIntervals(std::vector<ompl::base::State*> &mainPath) // finds intervals of same action index, builds shortcuts
{
    int prev_index = getChangedIndex(mainPath.at(0),mainPath.at(1));
    std::vector<int> transitions;

    for (size_t i = 1; i < mainPath.size()-1 ; ++i) {
        if(prev_index != getChangedIndex(mainPath.at(i),mainPath.at(i+1)))
        {
            transitions.push_back(i);
            prev_index = getChangedIndex(mainPath.at(i),mainPath.at(i+1));
        }
        else
            si_->freeState(mainPath.at(i));
    }

    std::vector<ompl::base::State *> simplifiedPath;

    simplifiedPath.push_back(mainPath.at(0));
    for(auto &index : transitions){
//        if(!si_->equalStates(mainPath.at(index),simplifiedPath.back()))
        simplifiedPath.push_back(mainPath.at(index));
//        else{ // should actually never occur
//            std::cout<<"test"<<std::endl;
//            si_->freeState(mainPath.at(index));
//        }
    }

    simplifiedPath.push_back(mainPath.back());
    mainPath.clear();
    mainPath = simplifiedPath;

}


void ompl::geometric::RRTnew::cutOffIfGoalReached(std::vector<ompl::base::State*> &mainPath)
{
    for(size_t i = mainPath.size()-1 ; i > 0 ; i--)
    {
        if(getChangedIndex(mainPath.at(i),mainPath.at(i-1)) != goalIndex_)
        {
            si_->freeState(mainPath.at(i));
            mainPath.erase(mainPath.begin()+i);
        }
        else
            break;
    }
}

ompl::geometric::RRTnew::GrowState ompl::geometric::RRTnew::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                     Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;
    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    si_->getStateSpace()->interpolate(nmotion->state, rmotion->state,maxDistance_, tgi.xstate);
    // statt maxdistance / d, auf maxdistance weil sonst kommt er nicht weiter

    if (si_->equalStates(nmotion->state, tgi.xstate))
        return TRAPPED;

    dstate = tgi.xstate;

    if(d>maxDistance_){
        reach = false;
    }

    if (si_->equalStates(nmotion->state, dstate))
        return TRAPPED;


    if(!validMotionCheck(tgi.start,nmotion->state,dstate)){

        return TRAPPED;
    }

    auto newCost = opt_->motionCost(nmotion->state,dstate);

    if(newCost.value()  > 1.0){ // bestcost = 1.0, state transition already in only one dimension, no isolation needed

        if(useIsolation_ )
        {
            std::vector<ompl::base::State *> dstates;
            std::vector<double> s1,s2;
            si_->getStateSpace()->copyToReals(s1,nmotion->state);
            si_->getStateSpace()->copyToReals(s2,dstate);
            std::vector<int> g1 = getChangedGroups(s1,s2);
            buildIsoStates(nmotion->state,dstate,g1,dstates);

            std::vector<Motion *> stack_motion;
            if (dstates.size() == 0) { // konnte nichts isolieren
                //     std::cout << "err " << std::endl;
                return TRAPPED;
            }

            Motion *premotion = nmotion;
            for (auto st: dstates) { // TODO FEHLER HIER

                auto *motion = new Motion(si_);
                createNewMotion(st, premotion,motion);

                stack_motion.push_back(motion);
                premotion = motion;

            }

            for (auto const &mot_: stack_motion) { // add motions at the end only if states could be isolated, only if loop passed without trapped
                tree->add(mot_);
                incCost = opt_->combineCosts(incCost, base::Cost(1.0)); // todo ?
            }
            freeStates(dstates);

            tgi.xmotion = stack_motion.back();

            return reach ? REACHED : ADVANCED;
        }
        return TRAPPED;
    }

    incCost = opt_->combineCosts(incCost,opt_->motionCost(nmotion->state,dstate )); // todo
    auto * motion = new Motion(si_);
    createNewMotion(dstate,nmotion,motion);
    tree->add(motion);
    tgi.xmotion = motion;

    return reach ? REACHED : ADVANCED;
}

void ompl::geometric::RRTnew::simplifyPath(std::vector<ompl::base::State *> &path)
{

    int counter =0;
    base::Cost maxCost_(getCostPath(path));

    while(true){
        pathDefrag(path);
        base::Cost temp_cost(getCostPath(path));
         if(temp_cost.value() <= maxCost_.value())
        {
            if(temp_cost.value() == maxCost_.value())
                counter++;
            if(counter ==2) // if the cost stays the same twice,
                return;
            maxCost_ = temp_cost;
        }
        else
            return;
    }

}

void ompl::geometric::RRTnew::checkRepairPath(std::vector<ompl::base::State *> &path_)
{

    for(size_t i = 0; i < path_.size()-1; i++)
    {
        if(si_->equalStates(path_.at(i),path_.at(i+1))){  // equal states may be added somewhere else (outside growstate, i couldnt find where)
            path_.erase(path_.begin()+i);
        }

        if(opt_->motionCost(path_.at(i),path_.at(i+1)).value()> 1.0) // path defrag needs every state with motion cost 1.0
         {
             std::vector<ompl::base::State *> iso_;
             isolateStates(path_.at(i),path_.at(i+1),iso_);
             bool check = true;
             if(check){
                 path_.insert(path_.begin()+i+1,iso_.begin(),iso_.end());
                 //(todo/further improvement)check collision if yes new interpolate?/replan sub path?
             }
         }
    }
}



void ompl::geometric::RRTnew::constructSolutionPath(ompl::geometric::PathGeometric &path, Motion * startMotion, Motion * goalMotion)
{
    if (startMotion->parent != nullptr)
        startMotion = startMotion->parent;
    else
        goalMotion = goalMotion->parent;

    connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

    std::vector<Motion *> mpath1;
    getMotionVectors(startMotion,mpath1);
    std::vector<Motion *> mpath2;
    getMotionVectors(goalMotion,mpath2);


    path.getStates().reserve(mpath1.size() + mpath2.size());
    for (int i = mpath1.size() - 1; i >= 0; --i){
        path.append(mpath1[i]->state);
    }
    for (auto &i : mpath2){
        path.append(i->state);
    }

}

ompl::base::PlannerStatus ompl::geometric::RRTnew::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        motion->cost = opt_->identityCost();
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool solved = false;

    auto best_path(std::make_shared<PathGeometric>(si_));
    bestCost_ = opt_->infiniteCost();


    while (!ptc)
    {
        iterations_++;
        TreeData &tree = startTree_ ? tStart_ : tGoal_;
        tgi.start = startTree_;
        startTree_ = !startTree_;
        TreeData &otherTree = startTree_ ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                motion->cost = opt_->identityCost();
                motion->index_changed = 0;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }
        /* sample random state */

        sampler_->sampleUniform(rstate);

        GrowState gs = growTree(tree, tgi, rmotion);


        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */
            /* if reached, it means we used rstate directly, no need to copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            GrowState gsc = ADVANCED;
            tgi.start = startTree_;

            while (gsc == ADVANCED)
                gsc = growTree(otherTree, tgi, rmotion);

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            Motion *startMotion = tgi.start ? tgi.xmotion : addedMotion;
            Motion *goalMotion = tgi.start ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root) )
            {

                auto path(std::make_shared<PathGeometric>(si_));
                constructSolutionPath(*path,startMotion,goalMotion);

                checkRepairPath(path->getStates());

                simplifyPath(path->getStates());

                cutOffIfGoalReached(path->getStates());

                size_t prev_size;
                int k = 1;
                do{
                    k++;
                    prev_size = path->getStateCount();
                    trySkipFragment(path->getStates());
                }while(path->getStateCount() < prev_size );

//                shortcutToNextGoalFragment(path->getStates());

                simplifyActionIntervals(path->getStates());

                if(getCostPath(path->getStates()) < bestCost_.value())
                {
                    OMPL_DEBUG("Better path found, new path cost: %d, old path cost: %d", getCostPath(path->getStates()), int(bestCost_.value()));
                    freeStates(best_path->getStates());
                    best_path = path;
                    bestCost_ = base::Cost(getCostPath(best_path->getStates()));
                }
                else
                {
              //      std::cout << "worst path :" << getCostPath(path->getStates()) << ", best current : " << bestCost_.value() << std::endl;
                }

                if(ptc)
                {
                    pdef_->addSolutionPath(best_path, false, 0.0, getName());
                    solved = true;
                    break;
                }
                else
                {
                    continue;
                }

            }
            else
            {
                if(ptc && best_path->getStates().size() > 1)
                {
                    pdef_->addSolutionPath(best_path, false, 0.0, getName());
                    solved = true;
                    break;

                }
                else if (tgi.start)
                {
                    // We were working from the startTree.
                    double dist = 0.0;
                    goal->isSatisfied(tgi.xmotion->state, &dist);
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = tgi.xmotion;
                    }
                }
            }
        }
        //  tgi.reverseflag = !tgi.reverseflag;


    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    if(best_path->getStateCount()>1)
    {
        pdef_->addSolutionPath(best_path, false, 0.0, getName());
        solved = true;
    }
    else if (approxsol && !solved)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (approxsol != nullptr)
        {
            mpath.push_back(approxsol);
            approxsol = approxsol->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, true, approxdif, getName());
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::RRTnew::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

    // Add some info.
    data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}

std::string ompl::geometric::RRTnew::bestCostProgressProperty() const
{
    return std::to_string(this->bestCost_.value());
}