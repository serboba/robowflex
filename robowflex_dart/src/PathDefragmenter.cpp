//
// Created by serboba on 17.04.22.
//

#include <robowflex_dart/PathDefragmenter.h>
#include <robowflex_dart/IsoManipulationOptimization.h>

ompl::geometric::PathDefragmenter::PathDefragmenter(base::SpaceInformationPtr si,
                                                    std::vector<std::vector<int>> fragment_indices, int goalIndex, const base::OptimizationObjectivePtr &opt)
                                                    :si_(std::move(si)), fragment_indices(fragment_indices)
                                                    {
    goalIndex_ = goalIndex;

    opt_ = std::make_shared<base::IsoManipulationOptimization>(si_,fragment_indices);

                                                    }




std::vector<int> ompl::geometric::PathDefragmenter::getChangedGroups(const std::vector<double> &from_,const std::vector<double> &to_){
    std::vector<int> groups;
    for(size_t i = 0 ; i < fragment_indices.size(); i++){
        for(auto index : fragment_indices.at(i)){
            if(abs(from_.at(index)-to_.at(index))  > 0.0001){
                groups.push_back(i);
                break;
            }
        }
    }
    return groups;
}

void ompl::geometric::PathDefragmenter::buildIsoStates(const std::vector<double> &from_,const std::vector<double> &to_,
                                             std::vector<int> &changed_index_groups,
                                             std::vector<ompl::base::State* > &iso_ ) {
    std::vector<double> intermediate_st(from_.size());
    intermediate_st = from_;


    for (size_t v = 0; v < changed_index_groups.size(); v++) {
        for (int i: fragment_indices.at(changed_index_groups.at(v))) {
            intermediate_st[i] = to_[i];
        }
        base::State *temp = si_->getStateSpace()->allocState(); // todo mem leak?
        si_->getStateSpace()->copyFromReals(temp, intermediate_st);
        iso_.push_back(temp);
    }
}


void ompl::geometric::PathDefragmenter::isolateStates(const base::State* rfrom, const base::State* rto, std::vector<ompl::base::State*> &iso_){

    std::vector<double> from_,to_;
    si_->getStateSpace()->copyToReals(from_,rfrom);
    si_->getStateSpace()->copyToReals(to_,rto);

    std::vector<int> groups = getChangedGroups(from_,to_);
    buildIsoStates(from_,to_,groups,iso_);
}


void ompl::geometric::PathDefragmenter::getIntermediateState(const ompl::base::State *from,const ompl::base::State * to, ompl::base::State *state, int index_group)
{
    std::vector<double> s_new_v,to_;

    si_->getStateSpace()->copyToReals(s_new_v,from);
    si_->getStateSpace()->copyToReals(to_,to);

    std::vector<int> group = fragment_indices.at(index_group);
    for(size_t i = 0; i< group.size(); i++){
        s_new_v[group.at(i)] = to_[group.at(i)];
    }

    si_->getStateSpace()->copyFromReals(state,s_new_v);
}


int ompl::geometric::PathDefragmenter::getChangedIndex(const ompl::base::State *from,const ompl::base::State * to){

    // setze voraus dass cost 1 zw. from und to, keine equal states

    std::vector<double> s_from,s_to;
    si_->getStateSpace()->copyToReals(s_from,from);
    si_->getStateSpace()->copyToReals(s_to,to);

    for(size_t i = 0; i < fragment_indices.size() ; i++)
    {
        for(auto const &index_in_gr : fragment_indices.at(i))
        {
            if(abs(s_from.at(index_in_gr) - s_to.at(index_in_gr)) > 1e-10)
                return i;
        }
    }
    return -1;
}


void ompl::geometric::PathDefragmenter::freeStates(std::vector<ompl::base::State*> &states)
{
    for(auto &st : states)
        si_->freeState(st);
    states.clear();
}

void ompl::geometric::PathDefragmenter::freeStates(std::vector<std::pair<ompl::base::State *,int >> &states)
{
    for(auto &st : states)
        si_->freeState(st.first);
    states.clear();
}

bool ompl::geometric::PathDefragmenter::reConnect(ompl::base::State *from,
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


void ompl::geometric::PathDefragmenter::reConnect(ompl::base::State *from, std::vector<std::pair<ompl::base::State *,int >> &prio_,
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

void ompl::geometric::PathDefragmenter::findNextFragment(int start_index, int prev_index, std::vector<ompl::base::State *> &mainPath, bool sameFragmentType,
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

int ompl::geometric::PathDefragmenter::pathDefrag(std::vector<ompl::base::State *> &mainPath) {

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

int ompl::geometric::PathDefragmenter::getCostPath(std::vector<ompl::base::State * > &states_)
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



void ompl::geometric::PathDefragmenter::getFragmentIDs(std::vector<ompl::base::State*> &path,std::vector<FragmentIntervals> &fragmentIDs, bool goalFragment = false)
{
    fragmentIDs.clear();
    int prev_index = getChangedIndex(path.at(0),path.at(1));

    for(size_t i = 1; i < path.size()-1; i++)
    {
        if(prev_index != getChangedIndex(path.at(i),path.at(i+1)))
        {
            // check for equal states, that may occur if we skip fragments
            if(si_->equalStates(path.at(i),path.at(i+1))){
//                std::cout<<"fragment equal states!! fixed!" << std::endl;
                path.erase(path.begin()+(i+1));
                continue;
            }
            if(fragmentIDs.size()>0)
                fragmentIDs.push_back(FragmentIntervals(fragmentIDs.back().end_index+1,i, prev_index));
            else
                fragmentIDs.push_back(FragmentIntervals(0,i, prev_index));

            prev_index = getChangedIndex(path.at(i),path.at(i+1));
        }
    }

    if(prev_index != fragmentIDs.back().id)
        fragmentIDs.push_back(FragmentIntervals(fragmentIDs.back().end_index+1,path.size()-1, prev_index));

    if(goalFragment)
    {
        std::vector<FragmentIntervals> goalfrags;
        for(size_t i = 0; i < fragmentIDs.size(); i++)
        {
            if(fragmentIDs.at(i).id == goalIndex_)
                goalfrags.push_back(fragmentIDs.at(i));
        }
        fragmentIDs = goalfrags;
    }
}

void ompl::geometric::PathDefragmenter::getFragment(int start_index, int end_index,
                                          std::vector<ompl::base::State*> &mainPath, std::vector<std::pair<ompl::base::State *,int>> &fragment)
{
    for(int j = start_index; j< end_index; j++){
        ompl::base::State * temp = si_->allocState();
        si_->getStateSpace()->copyState(temp,mainPath.at(j));
        fragment.push_back(std::make_pair(temp, getChangedIndex(mainPath.at(j-1),mainPath.at(j))));
    }
}


void ompl::geometric::PathDefragmenter::skipFragments(std::vector<ompl::base::State*> &mainPath)
{
    if(mainPath.size() < 3)
        return;
    std::vector<FragmentIntervals> fragmentIDs;
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

            if(mainPath.size() < 3)
                return;
        }
        else
            si_->freeState(temp_from);
        // no reconnection / jump over possible
    }

}


void ompl::geometric::PathDefragmenter::trySkipFragment(std::vector<ompl::base::State *> &mainPath) {


    size_t prev_size;

    do{
        prev_size = mainPath.size();
        skipFragments(mainPath);
    }while(mainPath.size() < prev_size );

}


void ompl::geometric::PathDefragmenter::simplifyActionIntervals(std::vector<ompl::base::State*> &mainPath) // finds intervals of same action index, builds shortcuts
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
//            si_->freeState(mainPath.at(index));
//        }
    }

    simplifiedPath.push_back(mainPath.back());
    mainPath.clear();
    mainPath = simplifiedPath;

}


void ompl::geometric::PathDefragmenter::cutOffIfGoalReached(std::vector<ompl::base::State*> &mainPath)
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

void ompl::geometric::PathDefragmenter::startPathDefrag(std::vector<ompl::base::State *> &path)
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

void ompl::geometric::PathDefragmenter::checkRepairPath(std::vector<ompl::base::State *> &path_)
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



void ompl::geometric::PathDefragmenter::doPathDefragComplete(std::vector<ompl::base::State *> &path_)
{
    checkRepairPath(path_);

    startPathDefrag(path_);

    cutOffIfGoalReached(path_);

    trySkipFragment(path_);

    simplifyActionIntervals(path_);

}
