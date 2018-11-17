#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <utility>

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
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_ininitial_conditions() const
    {
        return this->initial_conditions;
    }
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_conditions() const
    {
        return this->goal_conditions;
    }
    unordered_set<Action, ActionHasher, ActionComparator> get_actions() const 
    {
        return actions;
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
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

class GroundedAction
{
private:
    string name;
    list<string> arg_values;

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

// a node consists of several grounded conditions 
class node{
protected:
	// several grounded conditions that make a node
	unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> data;
	// parent of node
	node* parent;
public:
	node();
	node(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> vec);
	void setData(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> vec);
	unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> getData() {
        return data;
    }
	void setParent(node* rNode);
	node* getParent() {return parent;}
};
node::node() {}
node::node(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> vec) {
	setData(vec);
}
void node::setData(unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> vec) {
	data = vec;
}
void node::setParent(node* rNode) {
	parent = rNode;
}

// calculate the heuristic
double calHeu(node* n1, node* n2){
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions, goal_conditions;
    initial_conditions = n1->getData();
    goal_conditions = n2->getData();
    int unmatched = goal_conditions.size();
    for (GroundedCondition ggc : goal_conditions) {
        for (GroundedCondition igc : initial_conditions) {
            if (igc == ggc) {
                unmatched -= 1;
                break;
            }
        }
    }
    return unmatched;
}

// calculate child node using node and grounded action
node* childNode(node* nIn, Action aIn, list<string> str) {
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> nodeCondition;
    nodeCondition = nIn->getData();
    unordered_set<Condition, ConditionHasher, ConditionComparator> actionPrecondition;
    actionPrecondition = aIn.get_preconditions();
    // map from action to grounded
    unordered_map<string, string> mapToGround;
    for (string sybIn : aIn.get_args()) {
        mapToGround[sybIn] = str.front();
        str.pop_front();
    }
    // if node condition don't meet precondition, return NULL
    for (Condition cdt : actionPrecondition) {
        list<string> mappedCdt;
        for (string sybIn : cdt.get_args()) {
            if (mapToGround.find(sybIn)!=mapToGround.end()) {
                mappedCdt.push_back(mapToGround[sybIn]);
            }
            else {
                mappedCdt.push_back(sybIn);
            }
        }
        bool meetCondition = !cdt.get_truth();
        GroundedCondition agCdt(cdt.get_predicate(), mappedCdt);
        for (GroundedCondition ngCdt : nodeCondition) {
            if (agCdt == ngCdt) {
                meetCondition = !meetCondition;
                break;
            }
        }
        if (!meetCondition) {
            return NULL;
        }
    }
    // else, apply the action and return child node
    unordered_set<Condition, ConditionHasher, ConditionComparator> actionEffect;
    actionEffect = aIn.get_effects();
    for (Condition cdt : actionEffect) {
        list<string> mappedCdt;
        for (string sybIn : cdt.get_args()) {
            if (mapToGround.find(sybIn)!=mapToGround.end()) {
                mappedCdt.push_back(mapToGround[sybIn]);
            }
            else {
                mappedCdt.push_back(sybIn);
            }
        }
        GroundedCondition agCdt(cdt.get_predicate(), mappedCdt);
        // if truth, add condition
        if (cdt.get_truth()) {
            nodeCondition.insert(agCdt);
        }
        // else, erase action 
        else {
            nodeCondition.erase(agCdt);
        }
    }
    node* cNode = new node(nodeCondition);
    cNode->setParent(nIn);
    return cNode;
}

vector<string> combination;
vector<list<string>> allComb;
// calculate all combination
void go(int offset, int k, vector<string> asb) {
  if (k == 0) {
    vector<string> v = combination;
    sort(v.begin(), v.end());
    do {
        //static int count = 0;
        //cout << "combination no " << (++count) << ": [ ";
        list<string> vec;
        for (int i = 0; i < v.size(); ++i) { 
            //cout << v[i] << " "; 
            vec.push_back(v[i]);
        }
        //cout << "] " << endl;
        allComb.push_back(vec);
    } while( next_permutation(v.begin(), v.end()) );
    return;
  }
  for (int i = offset; i <= asb.size() - k; ++i) {
    combination.push_back(asb[i]);
    go(i+1, k-1, asb);
    combination.pop_back();
  }
}

list<GroundedAction> planner(Env* env)
{
    // this is where you insert your planner
    list<GroundedAction> thePath;
    vector<node*> recordPath;
    
    // start and goal 
    node* start = new node(env->get_ininitial_conditions());
    node* goal = new node(env->get_goal_conditions());
    
    // set openList<f, node*> and insert start into it
	set<pair<double, node*>> openList;
	openList.insert(make_pair(calHeu(start, goal), start));
	// unordered_map<node*, <<g, if closed>, GroundedAction>> for future back track
	unordered_map<node*, pair<pair<double, bool>, GroundedAction*> > reachedNode;
    GroundedAction* startGa = new GroundedAction("start", {"nothing"});
	reachedNode[start] = make_pair(make_pair(0.0, false), startGa);
	int countInWhile = 0;
    //thePath.push_front(*startGa);
    
    // all actions and all symbles
    unordered_set<Action, ActionHasher, ActionComparator> allAct = env->get_actions();
    vector<string> allSyb(env->get_symbols().begin(), env->get_symbols().end()) ;
// for (string str:allSyb) cout<< str << " ";
// cout<< endl;
	// expand until openList is empty or goal reached
	while(!openList.empty()) {
		countInWhile++;
		pair<double, node*> bestCell = *openList.begin();
		openList.erase(openList.begin());
		reachedNode[bestCell.second].first.second = true;
// // movement in close list
// cout<< endl << reachedNode[bestCell.second].second->get_name()<<" ";
// for (string str : reachedNode[bestCell.second].second->get_arg_values()) {
    // cout<<str<<" ";
// }
// cout<<endl;
// for (GroundedCondition gcd : bestCell.second->getData()) {
    // cout <<" "<< gcd.get_predicate()<<" ";
    // for (string str : gcd.get_arg_values()) {
        // cout<<str<<" ";
    // }
    // cout<<",";
// }
// cout<<endl;
		/*----------------if goal reached ,break--------------------*/
		if (calHeu(bestCell.second, goal)==0.0) {
			printf("\n\nreach goal!!!\n\n");
			node* backTrackNode = bestCell.second;
			while (1) {
// // condition after each movement
// cout << endl;
                // for (GroundedCondition gcd : backTrackNode->getData()) {
    // cout << "" << gcd.get_predicate()<<" ";
    // for (string str : gcd.get_arg_values()) {
        // cout<<str<<" ";
    // }
    // cout<<", ";
// }
// cout<<endl;
				thePath.push_front(*reachedNode[backTrackNode].second);
				backTrackNode = backTrackNode->getParent();
				if (reachedNode[backTrackNode].second->get_name().compare("start")==0) {break;}
			}
			break;
		}
		
		
		// for the best node, add all it's successors into openList
        // for all action
		for (Action act : allAct) {
            // num of args taken by action
            int numOfSyb = act.get_args().size();
// cout<< "num of syb: " << numOfSyb << endl;
            // get all possible combination from allSyb, store in allComb
            allComb.clear();
            go(0, numOfSyb, allSyb);
// cout<<allComb.size()<<" "<< allComb.front().size()<<endl;
            for (list<string> comb : allComb) {
                // form an action to check if valid successor
                node* tmpt = childNode(bestCell.second, act, comb);
                // if valid successor
                if (tmpt != NULL) {
// cout<<"     "<< act.get_name()<<" ";
// for (string str : comb) {
    // cout<<str<<" ";
// }
// cout<<endl;
// // condition of successor
// for (GroundedCondition gcd : tmpt->getData()) {
    // cout << "     "<< gcd.get_predicate()<<" ";
    // for (string str : gcd.get_arg_values()) {
        // cout<<str<<" ";
    // }
    // cout<<",";
// }
// cout<<endl;
                    double newG = reachedNode[bestCell.second].first.first + 1.0;
                    double newF = newG + calHeu(tmpt, goal);
                    bool reached = false;
                    for (pair<node*, pair<pair<double, bool>, GroundedAction*> > aNode : reachedNode) {
                       // if found, already reached
                       if (calHeu(tmpt, aNode.first) == 0.0 && calHeu(aNode.first, tmpt) == 0.0) {
                           reached = true;
                           // if not closed, compare and update
                           if (!aNode.second.first.second) {
                                if (newG < aNode.second.first.first) {
                                    aNode.second.first.first = newG;
                                    aNode.second.second = new GroundedAction(act.get_name(), comb);
                                    aNode.first->setParent(bestCell.second);
// cout<<"      updated"<<endl;
                                }
// cout<<"      no better"<<endl;
                           }
                           // else, closed, do nothing
                           else {
// cout<<"               closed"<<endl;
                           }
                           break;
                       }
                    } 
                    // see if not found, add to open
                    if (!reached) {
                        reachedNode[tmpt] = make_pair(make_pair(newG, false), new GroundedAction(act.get_name(), comb));
                        openList.insert(make_pair(newF, tmpt));
// cout<<"      added"<<endl;
                    }
                }
            }
            
		}
        
        if (countInWhile >= 500000) break;
	}
    
    
    
    
    // blocks world example
    list<GroundedAction> actions;
    actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));

    return thePath;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
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