#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <stdexcept>
#include <time.h>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    // GroundedCondition(const Condition& c) {
    //     this->predicate = c.get_predicate();
    //     this->truth = c.get_truth();
    //     for (string l : c.get_args())
    //     {
    //         this->arg_values.push_back(l);
    //     }
    // }
    
    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }
    void set_truth(bool t) {
        this->truth = t;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gPreconditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gEffects;

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> getPreconditions() const {
        return this->gPreconditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> getEffects() const {
        return this->gEffects;
    }
    
    bool meetPreconditions(const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& cond) const {
        for (const auto& pre_cond : this->gPreconditions) {
            if (cond.find(pre_cond) == cond.end()) {
                // cout << "Cannot find " << pre_cond << "in : ";
                // for (const auto& c : cond) {
                //     cout << c;
                // }
                // cout << endl;
                return false;
            }
            else {
                // cout << "Found " << pre_cond << endl;
            }
        }
        return true;
    }

    void addGroundedPrecondition(const GroundedCondition& gc) {
        this->gPreconditions.insert(gc);
    }

    void addGroundedEffect(const GroundedCondition& gc) {
        this->gEffects.insert(gc);
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }
    unordered_set<Action, ActionHasher, ActionComparator> get_actions() const
    {
        return this->actions;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_conditions() const
    {
        return this->initial_conditions;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_conditions() const
    {
        return this->goal_conditions;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

using Args = list<string>;
using ArgsPermutation = list<Args>;
using AllArgsPermutation = vector<ArgsPermutation>;

void printPermutations(const AllArgsPermutation& permutations) {
    for (int i=0; i<permutations.size(); ++i) {
        cout << "Permutation[" << i << "]:" << endl;
        for (const auto& args : permutations[i]) {
            for (const auto& s : args) {
                cout << s << " ";
            }
            cout << endl;
        }
    }

}

void DFS(int depth, vector<bool>& visited, Args curr_args, AllArgsPermutation& all_permutations, const vector<string>& symbols) {
    if (depth > symbols.size()) {
        return;
    }

    for (int i=0; i<symbols.size(); ++i) {
        if (visited[i] == false) {
            visited[i] = true;
            curr_args.push_back(symbols[i]);
            all_permutations[depth].push_back(curr_args);

            DFS(depth+1, visited, curr_args, all_permutations, symbols);

            visited[i] = false;
            curr_args.pop_back();
        }
    }
}


  
AllArgsPermutation getAllPermutation(const unordered_set<string>& symbols) {
    vector<string> symbols_vec;
    for (const auto& s : symbols) {
        symbols_vec.push_back(s);
    }
    
    AllArgsPermutation result;
    // result.resize(symbols.size());
    for (int i=0; i<symbols.size(); ++i) {
        ArgsPermutation p;
        result.push_back(p);
    }

    int depth = 0;
    vector<bool> visited(symbols.size(), false);
    Args curr_args;

    DFS(depth, visited, curr_args, result, symbols_vec);

    return result; 
}


list<GroundedAction> generateAllGroundedActions(const unordered_set<Action, ActionHasher, ActionComparator>& actions, const unordered_set<string>& symbols) {
    const auto& all_permutations = getAllPermutation(symbols);

    // printPermutations(all_permutations);

    list<GroundedAction> all_grounded_actions;
    for (const auto& action : actions) {
        int num_args = action.get_args().size();

        for (const auto& args : all_permutations[num_args-1]) {

            // construct a grounded action based on a specific permutation of arguments
            GroundedAction ga(action.get_name(), args);

            // ground the preconditions and effects of this grounded action
            const auto& preconditions = action.get_preconditions();
            const auto& effects = action.get_effects();
            const auto& arg_names = action.get_args();

            // create args mapping
            unordered_map<string, string> arg_mapping;
            for (const auto& sym : symbols) {
                arg_mapping[sym] = sym;
            }
            auto itr = args.begin();
            for (const auto& arg_name : arg_names) {
                arg_mapping[arg_name] = *itr;
                ++itr;
            }

            // ground preconditions
            for (const auto& pc : preconditions) {
                list<string> mapped_args;
                for (const auto& a : pc.get_args()) {
                    mapped_args.push_back(arg_mapping[a]);
                }
                GroundedCondition gc(pc.get_predicate(), mapped_args, pc.get_truth());
                ga.addGroundedPrecondition(gc);
            }

            // ground effects
            for (const auto& e : effects) {
                list<string> mapped_args;
                for (const auto& a : e.get_args()) {
                    mapped_args.push_back(arg_mapping[a]);
                }
                GroundedCondition ge(e.get_predicate(), mapped_args, e.get_truth());
                ga.addGroundedEffect(ge);
            }
            
            // cout << ga << endl;
            all_grounded_actions.emplace_back(ga);


        }
    }
    return all_grounded_actions;
}

class State {

public:
    // Constructors
    State() {}
    State(const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& cond) {
        setConditions(cond);
    }

    // Setter function
    void setConditions(const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& cond) {
        this->conditions = cond;
    }

    bool isValid() {
        return this->valid;
    }

    State branch(const GroundedAction& action) {
        // given an action, branch the next state from the current state
        State new_state(conditions);
        new_state.valid = action.meetPreconditions(conditions);
        if (new_state.valid) {
            // cout << "valid" << endl << endl << endl << endl;
            for (auto effect : action.getEffects()) {
                if (!effect.get_truth()) {
                    // cout << "erased" << effect << endl;
                    effect.set_truth(true);
                    new_state.conditions.erase(effect);
                }
                else {
                    // cout << "inserted" << effect << endl;
                    new_state.conditions.insert(effect);
                }
            }
        }
        return new_state;

    }

    // Members
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> conditions;
    bool valid;

};

int getHeuristic(const State& s, const State& goal) {
    // number of unsatisfsied conditions in goal state
    int h = goal.conditions.size();
    for (const auto& c1 : goal.conditions) {
        // for each condition in goal state, search for already satisfied ones in start state
        for (const auto& c2 : s.conditions) {
            if (c1 == c2) {
                // a condition is satisfied
                --h;
                break;
            }
        }
    }
    return h;
}

class Node {
public:
    Node() {}
    Node(const State& state, int cost, int heuristic, int id) : state(state), cost(cost), heuristic(heuristic), id(id) {}


    State state;
    int cost;
    int heuristic;

    int id;

};

typedef std::pair<int, int> CostNodeId;
void addToOpenList(const Node& node, std::priority_queue<CostNodeId, std::vector<CostNodeId>, std::greater<CostNodeId> >& open_list) {
    open_list.push(make_pair(node.cost + node.heuristic, node.id));
}

list<GroundedAction> planner(Env* env)
{
    auto t_start = clock();
    
    // all possible grounded actions
    list<GroundedAction> all_actions = generateAllGroundedActions(env->get_actions(), env->get_symbols());
    // the list of actions to execute to get from start state to goal state
    list<GroundedAction> path;

    // // A* search
    std::priority_queue<CostNodeId, std::vector<CostNodeId>, std::greater<CostNodeId> > open_list;

    unordered_map<int, Node> graph;
    unordered_map<int, bool> closed_list;

    State start(env->get_initial_conditions());
    State goal(env->get_goal_conditions());
    // cout << "Start state: ";
    // for (const auto& cond : start.conditions) {
    //     cout << cond;
    // }
    // cout << endl;


    // for (const auto& action : all_actions) {
    //     auto next_state = start.branch(action);
    //     if (next_state.isValid()) {
    //         cout << "From state: ";
    //         for (const auto& cond : start.conditions) {
    //             cout << cond;
    //         }
    //         cout << endl;

    //         cout << "Applied action: " << action << endl;
    //         cout << "With effects: ";
    //         for (const auto& eff : action.getEffects()) {
    //             cout << eff;
    //         }
    //         cout << endl;
    //         cout << "To state: ";
    //         for (const auto& cond : next_state.conditions) {
    //             cout << cond;
    //         }
    //         cout << endl;
    //     }
    // }


    graph[0] = Node(start, 0, getHeuristic(start, goal), 0);
    addToOpenList(graph[0], open_list);
    // graph[1] = Node(goal, 0, getHeuristic(start, goal), 0);
    int counter = 0;
    while (!open_list.empty()) {
        cout << counter << endl;
        // get the node with least cost
        auto current_id = open_list.top().second;
        open_list.pop();
        closed_list[current_id] = true;

        auto& current_node = graph[current_id];
        if (current_node.heuristic == 0) {
            cout << "Goal found!" << endl;
            // back trace to find a path
            break; // or return path (actions)
        }

        // branch out from current node
        
        for (const auto& action : all_actions) {
            auto next_state = current_node.state.branch(action);
            if (next_state.isValid()) {
                counter++;
                // state, cost, heuristic, id
                int id = graph.size();
                Node new_node(next_state, current_node.cost + 1, getHeuristic(next_state, goal), id);
                graph[id] = new_node;
                addToOpenList(new_node, open_list);

                // cout << "From state: ";
                // for (const auto& cond : start.conditions) {
                //     cout << cond;
                // }
                // cout << endl;

                // cout << "Applied action: " << action << endl;
                // cout << "With effects: ";
                // for (const auto& eff : action.getEffects()) {
                //     cout << eff;
                // }
                // cout << endl;
                // cout << "To state: ";
                // for (const auto& cond : next_state.conditions) {
                //     cout << cond;
                // }
                // cout << endl;
            }
        }

        // loop through all possible actions
        // const auto& new_state = current_node.state.branch(grounded_action)

        // for every new state
        // calculate their heuristics, cost
        // put them into open list





    }


    // blocks world example
    
    // actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    // actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    // actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));
    auto t_end = (float)(clock() - t_start) / CLOCKS_PER_SEC;
    cout << "Time taken: " << t_end << " seconds" << endl;

    return path;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("p1.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}