# List of commands

### Skid sliding
- **method:** backstepping
- **is adaptive:** yes
- **is predictive:** no
- **robot models:** 4WS, 2TD
- **description:** backstepping adaptive approach for skid steering or tracked robot where the
  slippage is modeled by a linear speed disturbance, an angular speed disturbance and a drift
  angle on the linear speed vector
- **publication**: thesis of Luc Debos "Contribution à l'élaboration de lois de commandes
  génériques pour le suivi de trajectoire d'engins agricoles soumis à des dynamiques fortes et
  incertaines", section 2.2.6.1
- **gains:**
    * lateral deviation (unit: 1/m): negative factor applied to the lateral deviation on the
      spatial differential equation
    * course deviation (unit: 1/s): negative factor applied to the course deviation on the
      temporal differential equation. This coef should be greater than the previous one.
- **sliding model:**
    * sliding angle on linear speed vector
    * linear speed disturbance
    * angular speed disturbance



# List of sliding observers
