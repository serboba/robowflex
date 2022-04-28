//
// Created by serboba on 17.04.22.
//

#ifndef ROBOWFLEX_DART_PATHDEFRAGMENTER_H
#define ROBOWFLEX_DART_PATHDEFRAGMENTER_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>

namespace ompl
{
    namespace geometric {

        // OMPL_CLASS_FORWARD(PathDefragmenter)

        class PathDefragmenter {
        public:


            struct FragmentIntervals
            {
                int start_index;
                int end_index;
                int id;

                FragmentIntervals(int start_, int end_, int id_) : start_index(start_), end_index(end_),id(id_){}

            };


            PathDefragmenter(base::SpaceInformationPtr si, std::vector<std::vector<int>> fragment_indices, int goalIndex = 0,
                             const base::OptimizationObjectivePtr &opt = nullptr);



            void startPathDefrag(std::vector<ompl::base::State *> &path);

            void checkRepairPath(std::vector<ompl::base::State *> &path_);

            void trySkipFragment(std::vector<ompl::base::State *> &mainPath);

            int pathDefrag(std::vector<ompl::base::State *> &mainPath);

            void simplifyActionIntervals(std::vector<ompl::base::State *> &mainPath);

            void cutOffIfGoalReached(std::vector<ompl::base::State *> &mainPath);


            void doPathDefragComplete(std::vector<ompl::base::State *> &path_);


        private:


            void
            getIntermediateState(const ompl::base::State *from, const ompl::base::State *to, ompl::base::State *state,
                                 int index_group);


            int getChangedIndex(const ompl::base::State *from, const ompl::base::State *to);


            void freeStates(std::vector<ompl::base::State *> &states);

            void freeStates(std::vector<std::pair<ompl::base::State *, int>> &states);

            bool reConnect(base::State *from, std::vector<std::pair<ompl::base::State *, int>> &queue_,
                           std::vector<ompl::base::State *> &rewireResult);

            void reConnect(base::State *from, std::vector<std::pair<ompl::base::State *, int>> &prio_,
                           std::vector<std::pair<ompl::base::State *, int>> &stack_,
                           std::vector<ompl::base::State *> &rewireResult);

            void
            findNextFragment(int start_index, int prev_index, std::vector<ompl::base::State *> &mainPath,
                             bool sameFragmentType,
                             std::vector<std::pair<ompl::base::State *, int>> &foundFragment);


            int getCostPath(std::vector<ompl::base::State *> &states_);

            void getFragment(int start_index, int end_index, std::vector<ompl::base::State *> &mainPath,
                             std::vector<std::pair<ompl::base::State *, int>> &fragment);


            void getFragmentIDs(std::vector<ompl::base::State *> &path, std::vector<FragmentIntervals> &fragmentIDs,
                                bool goalFragment);


            void
            isolateStates(const base::State *rfrom, const base::State *rto, std::vector<ompl::base::State *> &iso_);

            std::vector<int> getChangedGroups(const std::vector<double> &from_, const std::vector<double> &to_);

            void
            buildIsoStates(const std::vector<double> &from_, const std::vector<double> &to_,
                           std::vector<int> &changed_index_groups,
                           std::vector<ompl::base::State *> &iso_);

            void skipFragments(std::vector<ompl::base::State *> &mainPath);



        protected:

            ompl::base::SpaceInformationPtr si_;
            std::vector<std::vector<int>> fragment_indices;
            int goalIndex_;

            base::OptimizationObjectivePtr opt_;

        };

    }
}










#endif //ROBOWFLEX_DART_PATHDEFRAGMENTER_H
