% Main solve predicate with energy limit
solve :-
    write('Enter the drone\'s initial energy (e.g., 6): '),
    read(X),
    integer(X),
    X > 0,                          %inputs the energy and starts the searching process
    initial_state(InitialState),
    goal_state(GoalState),
    calculateH(InitialState, GoalState, H),
    % Store initial energy for recharging
    retractall(solve_get_initial_energy(_)),
    assertz(solve_get_initial_energy(X)),     %places/stores the initial value before calling search
    search([ [InitialState, null, 0, H, H, X] ], [], GoalState).

% ========== STATE DEFINITIONS ==========
initial_state([
    [d, '-', p, '-', o],
    ['-', o, '-', '-', p],
    ['-', '-', o, p, '-'],
    [p, o, '-', 'R', '-'],  
    ['-', '-', p, o, '-']
]).


goal_state(GoalState) :- 
    initial_state(GoalState).

% ========== SEARCH CORE ==========
search(Open, Closed, Goal):-   %the base case, if the goal condition predicate is true
    getBestState(Open, [CurrentState, Parent, G, H, F, Energy], TmpOpen),
    goal_condition(CurrentState, Goal), !,
    write("Optimal Drone Route Found!"), nl, nl,
    solve_get_initial_energy(InitialEnergy),
    write("Initial Energy: "), write(InitialEnergy), nl, nl,
    printSolution([CurrentState, Parent, G, H, F, Energy], Closed),
    write("Final State:"), nl,
    printGrid(CurrentState).

search(Open, Closed, Goal):-
    getBestState(Open, CurrentNode, TmpOpen),  %finds the best state to go to 
    getAllValidChildren(CurrentNode, TmpOpen, Closed, Goal, Children),  %generates the possible legal/valid states
    addChildren(Children, TmpOpen, NewOpen),   %puts the children state in the open list to be considered 
    append(Closed, [CurrentNode], NewClosed),  %places the current state in the closed list to make it an illegal state
    search(NewOpen, NewClosed, Goal).  %recursive call to continue searching

% ========== GOAL CHECKING ==========
goal_condition(State, _) :-    % the predicate used to check whether the drone collected all packages or not
    flatten(State, FlatState),
    \+ member(p, FlatState).

% ========== HEURISTIC FUNCTION ==========
calculateH(State, _, H):-   %estimates the heuristic value, used to determine which node is better to move to and in pruning 
    flatten(State, FlatState),
    findall(X, (member(X, FlatState), X == p), Ps),
    length(Ps, H).

% ========== NODE SELECTION ==========
getBestState(Open, BestChild, Rest):-    %gets the best value according the value of the F which is calculated via the G and H
    findMin(Open, BestChild),
    delete(Open, BestChild, Rest).

findMin([X], X):- !.
findMin([Head|T], Min):-
    findMin(T, TmpMin),
    Head = [_, _, _, _, HeadF, _],
    TmpMin = [_, _, _, _, TmpF, _],
    (TmpF < HeadF -> Min = TmpMin ; Min = Head).

% ========== CHILD NODE GENERATION ==========
getAllValidChildren(Node, Open, Closed, Goal, Children):-  %gets all valid moves/states using the getNextState predicate
    findall(Next, getNextState(Node, Open, Closed, Goal, Next), Children).


getNextState([State, Parent, G, _, _, Energy], Open, Closed, Goal, [Next, State, NewG, NewH, NewF, NewEnergy]):-
    Energy > 0,  % only moves if there's energy left
    move(State, Next, MoveCost),   % moves and adjusts the energy the drone has
    TempEnergy is Energy - MoveCost,
    (is_drone_at_recharge(Next) ->        % Check if drone is at recharge station after the move
        solve_get_initial_energy(MaxEnergy),
        NewEnergy = MaxEnergy     %refills energy
    ;
        NewEnergy = TempEnergy           % normal case if it still has energy
    ),
    
    NewEnergy >= 0,       % checks if energy is not zero to determine whether to go on or not


    calculateH(Next, Goal, NewH),    %apply the A* calculations to determine the states to be pruned/ignored if visited
    NewG is G + MoveCost,
    NewF is NewG + NewH,
    (not(member([Next, _, _, _, _, _], Open)) ; memberButBetter(Next, Open, NewF, NewEnergy)),  %checks if a valid state will be pruned or not
    (not(member([Next, _, _, _, _, _], Closed)) ; memberButBetter(Next, Closed, NewF, NewEnergy)).  %checks if an invalid state was visited or not 

is_drone_at_recharge(State) :-   % Check if drone is at recharge station
    find_drone(State, (R, C)),
    initial_state(InitState),
    nth0(R, InitState, InitRow),
    nth0(C, InitRow, Cell),
    Cell == 'R'.

:- dynamic solve_get_initial_energy/1.    %stores the value dynamically for input
solve_get_initial_energy(6).  % Default value, will be updated during solve

memberButBetter(Next, List, NewF, NewEnergy):- %gets all valid states and checks them with better_state 
    findall([F, E], member([Next, _, _, _, F, E], List), Values),
    (Values = [] -> true ; 
        better_state(NewF, NewEnergy, Values)).


better_state(NewF, NewEnergy, Values) :-  %ensures that a state isn't revisited if it has a loweer F or same F but higehr energy
    \+ (member([F, E], Values), (F < NewF ; (F =:= NewF, E >= NewEnergy))).

addChildren(Children, Open, NewOpen):-  %places the valid states in the open list to use in search
    append(Open, Children, NewOpen).

% ========== SOLUTION PRINTING ==========
printSolution([State, null, G, H, F, Energy], _):-
    write("Initial State (g="), write(G), write(", h="), write(H), 
    write(", f="), write(F), write(", energy="), write(Energy), write("):"), nl,
    printGrid(State), nl, nl,
    write("Steps:"), nl, nl.

printSolution([State, Parent, G, H, F, Energy], Closed):-
    member([Parent, GrandParent, PrevG, PH, PF, PEnergy], Closed),
    printSolution([Parent, GrandParent, PrevG, PH, PF, PEnergy], Closed),
    write("Move (g="), write(G), write(", h="), write(H), 
    write(", f="), write(F), write(", energy="), write(Energy), write("):"), nl,
    printGrid(State), nl.

printGrid([]).
printGrid([H|T]) :-
    write('    '),
    write(H), nl,
    printGrid(T).

% ========== MOVEMENT RULES ==========
move(State, Next, 1):-  %handles the different legal moves 
    left(State, Next); right(State, Next); 
    up(State, Next); down(State, Next).

find_drone(State, (Row, Col)) :-  %lovates the drone's "d" location in the grid
    nth0(Row, State, RowList),
    nth0(Col, RowList, d).

  %all movements with checkers to avoid making an illegal move such as out of grid move or move into obstacle
% Left movement
left(State, Next):- 
    find_drone(State, (R, C)),
    C > 0,
    NewC is C - 1,
    nth0(R, State, Row),
    nth0(NewC, Row, Cell),
    Cell \= o,
    mark_position(State, R, C, NewC, Next).

% Right movement
right(State, Next):-
    find_drone(State, (R, C)),
    nth0(R, State, Row),
    length(Row, Cols),
    C < Cols - 1,
    NewC is C + 1,
    nth0(NewC, Row, Cell),
    Cell \= o,
    mark_position(State, R, C, NewC, Next).

% Up movement
up(State, Next):-
    find_drone(State, (R, C)),
    R > 0,
    NewR is R - 1,
    nth0(NewR, State, NewRow),
    nth0(C, NewRow, Cell),
    Cell \= o,
    mark_position_row(State, R, C, NewR, Next).

% Down movement
down(State, Next):-
    find_drone(State, (R, C)),
    length(State, Rows),
    R < Rows - 1,
    NewR is R + 1,
    nth0(NewR, State, NewRow),
    nth0(C, NewRow, Cell),
    Cell \= o,
    mark_position_row(State, R, C, NewR, Next).

% ========== GRID MODIFICATION ==========
mark_position(State, R, C, NewC, Next) :-   % handles modyifing the grid when the drones moves vertically
    initial_state(InitState),
    nth0(R, InitState, InitRow),
    nth0(C, InitRow, CellType),
    
    % If the current cell is recharge station in initial state, put R back in its place, otherwise make it empty
    (CellType == 'R' -> 
        substitute_in_grid(State, R, C, 'R', TempState)
    ;
        substitute_in_grid(State, R, C, '-', TempState)
    ),

    %puts drone in its place while keeping the special letter saved 
    nth0(R, TempState, TempRow),
    nth0(NewC, TempRow, DestContent),
    substitute_in_grid(TempState, R, NewC, d, Next).  

mark_position_row(State, R, C, NewR, Next) :-  %same for the above predicate but for horizontal moves
    initial_state(InitState),
    nth0(R, InitState, InitRow),
    nth0(C, InitRow, CellType),
        (CellType == 'R' -> 
        substitute_in_grid(State, R, C, 'R', TempState)
    ;
        substitute_in_grid(State, R, C, '-', TempState)
    ),
    
    nth0(NewR, TempState, TempRow),
    nth0(C, TempRow, DestContent),
    substitute_in_grid(TempState, NewR, C, d, Next).


substitute_in_grid(Grid, Row, Col, NewElement, NewGrid) :- %the actual modyifing logic used in the vertical movement predicate
    nth0(Row, Grid, OldRow),
    substitute_in_row(OldRow, Col, NewElement, NewRow),
    replace_nth0(Row, Grid, NewRow, NewGrid).

substitute_in_row(Row, Col, NewElement, NewRow) :- %the actual modyifing logic used in the horizontal movement predicate
    replace_nth0(Col, Row, NewElement, NewRow).


replace_nth0(0, [_|T], X, [X|T]) :- !.  %base case
replace_nth0(N, [H|T], X, [H|R]) :-  %replaces element at specific index in list
    N > 0,
    N1 is N - 1,
    replace_nth0(N1, T, X, R).
